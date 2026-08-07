"""Hybrid powertrain coordinator."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .battery import Battery, BatteryConfig
from .battery_thermal import BatteryThermal
from .battery_degradation import BatteryDegradation
from .motor import ElectricMotor, MotorConfig
from .regen_braking import RegenBraking, RegenConfig
from .hybrid_controller import HybridController
from .energy_manager import EnergyMode
from .awd_distribution import AWDDistributor
from .charging import Charger
from .hybrid_state import HybridState


@dataclass
class HybridConfig:
    enabled: bool = True
    architecture: str = "parallel"   # parallel | series | ev | series_parallel
    battery_capacity_kwh: float = 18.0
    motor_peak_power_kw: float = 150.0
    motor_peak_torque: float = 350.0
    ice_peak_torque: float = 320.0
    regen_enabled: bool = True
    energy_mode: str = "hybrid"
    rear_bias: float = 1.0
    soc0: float = 0.70


class HybridSolver:
    def __init__(self, config: HybridConfig | None = None):
        self.cfg = config or HybridConfig()
        bat_cfg = BatteryConfig(
            capacity_kwh=self.cfg.battery_capacity_kwh,
            soc0=self.cfg.soc0,
        )
        self.battery = Battery(bat_cfg)
        self.thermal = BatteryThermal()
        self.degradation = BatteryDegradation()
        self.motor = ElectricMotor(MotorConfig(
            peak_torque=self.cfg.motor_peak_torque,
            peak_power_kw=self.cfg.motor_peak_power_kw,
        ))
        self.regen = RegenBraking(RegenConfig(enabled=self.cfg.regen_enabled))
        try:
            emode = EnergyMode(self.cfg.energy_mode)
        except ValueError:
            emode = EnergyMode.HYBRID
        self.controller = HybridController(energy_mode=emode)
        self.awd = AWDDistributor(rear_bias=self.cfg.rear_bias)
        self.charger = Charger()
        self.state = HybridState(enabled=self.cfg.enabled)
        self._ice_on_time = 0.0

    def reset(self, soc: float | None = None) -> None:
        self.battery.reset(soc)
        self.thermal.temp_c = 25.0
        self.state = HybridState(enabled=self.cfg.enabled, battery_soc=self.battery.soc)

    def step(
        self,
        throttle: float = 0.0,
        brake: float = 0.0,
        wheel_speed: float = 0.0,
        dt: float = 0.01,
        *,
        charging: bool = False,
        charge_mode: str = "ac",
    ) -> HybridState:
        cfg = self.cfg
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))
        dt = float(max(dt, 1e-6))

        if not cfg.enabled:
            # Pure ICE pass-through proxy
            T_ice = throttle * cfg.ice_peak_torque
            self.state = HybridState(
                engine_torque=T_ice,
                motor_torque=0.0,
                wheel_torque=T_ice,
                battery_soc=self.battery.soc,
                engine_running=throttle > 0.05,
                mode="ice_only",
                enabled=False,
            )
            return self.state

        # Motor speed from wheel (assume final drive ~10 for simplicity if rpm not given)
        rpm = abs(wheel_speed) / 0.32 * 60.0 / (2.0 * np.pi) * 8.0  # rough FD
        rpm = float(np.clip(rpm, 0.0, self.motor.cfg.max_rpm))

        derate = self.thermal.derate()
        p_d, p_c = self.battery.available_power(derate)

        # Regen request
        T_regen, _ = self.regen.compute(brake, wheel_speed, rpm, p_c)

        # Controller
        cmd = self.controller.step(
            throttle, brake, self.battery.soc,
            ice_torque_cap=cfg.ice_peak_torque,
            motor_torque_cap=cfg.motor_peak_torque,
        )
        T_ice = cmd["ice_torque"]
        T_mot_req = cmd["motor_torque"] + T_regen

        if cfg.architecture == "ev":
            T_ice = 0.0
            cmd["engine_on"] = False

        # Motor
        mot = self.motor.step(T_mot_req, rpm, power_limit_kw=p_d if T_mot_req >= 0 else p_c)

        # Electrical power from motor (+ out of battery when motoring)
        p_elec = mot.power_elec_kw
        if charging:
            p_elec = self.charger.charge_power(charge_mode)

        bat = self.battery.step(p_elec, dt, temp_derate=derate)
        # Thermal: I²R loss approx
        loss_w = abs(bat.current) ** 2 * self.battery.cfg.r_internal
        self.thermal.step(loss_w, dt)
        self.degradation.update(abs(p_elec) * dt / 3600.0)

        T_f, T_r = self.awd.distribute(mot.torque)
        T_wheel = T_ice + T_f + T_r

        if cmd["engine_on"] and T_ice > 0:
            self._ice_on_time += dt

        self.state = HybridState(
            engine_torque=T_ice,
            motor_torque=mot.torque,
            wheel_torque=T_wheel,
            battery_soc=bat.soc,
            battery_voltage=bat.voltage,
            battery_current=bat.current,
            battery_temperature=self.thermal.temp_c,
            regen_power_kw=float(min(p_elec, 0.0)),
            motor_speed_rpm=rpm,
            motor_efficiency=mot.efficiency,
            engine_running=bool(cmd["engine_on"] and T_ice > 1.0),
            power_flow_kw=p_elec,
            energy_used_kwh=bat.energy_used_kwh,
            energy_recovered_kwh=bat.energy_recovered_kwh,
            mode=str(cmd["blend_mode"].value if hasattr(cmd["blend_mode"], "value") else cmd["blend_mode"]),
            enabled=True,
        )
        return self.state
