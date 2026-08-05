"""Powertrain coordinator: throttle → map → dynamics → output torque."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .engine import EngineConfig, EngineState
from .throttle import ThrottleModel, ThrottleState
from .flywheel import Flywheel
from .idle_controller import IdleController
from .rev_limiter import RevLimiter
from .engine_braking import engine_brake_torque
from .fuel_model import FuelModel, FuelState
from .thermal_model import ThermalModel, ThermalState


@dataclass
class PowertrainState:
    engine: EngineState = field(default_factory=EngineState)
    throttle: ThrottleState = field(default_factory=ThrottleState)
    fuel: FuelState = field(default_factory=FuelState)
    thermal: ThermalState = field(default_factory=ThermalState)
    load_torque: float = 0.0
    time: float = 0.0


class PowertrainSolver:
    def __init__(self, config: EngineConfig | None = None):
        self.cfg = config or EngineConfig()
        self.throttle_model = ThrottleModel()
        self.flywheel = Flywheel(inertia=self.cfg.inertia)
        self.idle = IdleController(idle_rpm=self.cfg.idle_rpm)
        self.limiter = RevLimiter(
            redline_rpm=self.cfg.redline_rpm,
            soft_start_rpm=self.cfg.soft_start_rpm,
            mode=self.cfg.limiter_mode,
        )
        self.fuel_model = FuelModel()
        self.thermal_model = ThermalModel()
        self.state = PowertrainState()
        self.state.engine.rpm = self.cfg.idle_rpm
        self.state.engine.omega = self.cfg.idle_rpm * 2.0 * np.pi / 60.0

    def reset(self) -> None:
        self.idle.reset()
        self.state = PowertrainState()
        self.state.engine.rpm = self.cfg.idle_rpm
        self.state.engine.omega = self.cfg.idle_rpm * 2.0 * np.pi / 60.0

    def step(
        self,
        throttle: float = 0.0,
        load_torque: float = 0.0,
        dt: float = 0.01,
    ) -> PowertrainState:
        """
        Advance powertrain by dt.

        load_torque: external load on crank (+ resists engine, N·m).
        Returns updated PowertrainState with engine.torque_output for downstream clutch.
        """
        cfg = self.cfg
        st = self.state

        if not cfg.enabled:
            st.engine = EngineState(rpm=0.0, omega=0.0)
            st.load_torque = load_torque
            st.time += dt
            return st

        # Throttle + idle assist
        thr_st = self.throttle_model.step(throttle, st.throttle, dt)
        assist = self.idle.assist(st.engine.rpm, thr_st.pedal, dt)
        thr_cmd = float(np.clip(thr_st.throttle + assist, 0.0, 1.0))
        thr_st = ThrottleState(pedal=thr_st.pedal, throttle=thr_cmd)

        emap = cfg.get_map()
        tq_ind = emap.torque_at(st.engine.rpm, thr_cmd)

        # Engine braking overlay (closed throttle)
        tq_eb = engine_brake_torque(
            st.engine.rpm, thr_cmd, idle_rpm=cfg.idle_rpm
        )
        # Map already has some closed-throttle negative; blend lightly
        if thr_cmd < 0.2:
            tq_ind = 0.5 * tq_ind + 0.5 * tq_eb

        # Thermal efficiency modifier
        tq_ind *= st.thermal.efficiency_factor

        # Rev limiter
        lim = self.limiter.factor(st.engine.rpm)
        tq_lim = tq_ind * lim

        # Viscous friction
        tq_fric = -cfg.friction_coeff * st.engine.omega

        # Net torque on inertia: indicated - load + friction
        # Convention: load_torque > 0 means load opposing engine
        tq_net = tq_lim + tq_fric - load_torque

        alpha = self.flywheel.alpha(tq_net)
        omega = st.engine.omega + alpha * dt
        # Stall / floor
        omega = max(omega, 0.0)
        rpm = omega * 60.0 / (2.0 * np.pi)

        # Hard stall recovery toward idle when very low and throttle present
        if rpm < cfg.stall_rpm and thr_cmd < 0.02:
            omega = 0.0
            rpm = 0.0
        elif rpm < cfg.idle_rpm * 0.5 and thr_cmd > 0.1:
            # Cranking-ish recovery
            omega = max(omega, cfg.idle_rpm * 0.5 * 2 * np.pi / 60)

        power_kw = max(tq_lim, 0.0) * omega / 1000.0

        fuel = self.fuel_model.step(
            power_kw, thr_cmd, rpm, cfg.redline_rpm, dt, st.fuel
        )
        thermal = self.thermal_model.step(power_kw, dt, st.thermal)

        eng = EngineState(
            rpm=float(rpm),
            omega=float(omega),
            torque_indicated=float(tq_ind),
            torque_brake=float(tq_eb),
            torque_output=float(tq_lim),  # shaft torque before external load
            power_kw=float(power_kw),
            throttle=thr_cmd,
            limiter_factor=float(lim),
        )

        self.state = PowertrainState(
            engine=eng,
            throttle=thr_st,
            fuel=fuel,
            thermal=thermal,
            load_torque=load_torque,
            time=st.time + dt,
        )
        return self.state
