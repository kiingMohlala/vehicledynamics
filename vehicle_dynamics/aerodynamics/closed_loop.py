"""
Phase 9.1/9.2 – Closed-loop aero with optional devices & pitch/heave dynamics.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .coefficients import AeroConfig
from .ride_height import RideHeightState
from .aero_solver import AeroResult, solve_aero
from .vehicle_interface import VehicleAeroInput, ride_from_vehicle_input
from .coupling import CoupledAxleLoads, couple_aero_to_tires
from .aero_devices import AeroDeviceConfig
from .aero_device_solver import AeroDeviceSolver, DeviceBreakdown


@dataclass
class PitchHeaveState:
    pitch: float = 0.0
    pitch_rate: float = 0.0
    heave: float = 0.0
    heave_rate: float = 0.0


@dataclass
class PitchHeaveParams:
    m: float = 1400.0
    Iy: float = 2200.0
    k_front: float = 35000.0
    k_rear: float = 40000.0
    c_front: float = 3500.0
    c_rear: float = 3500.0
    a: float = 1.35
    b: float = 1.35
    g: float = 9.81


@dataclass
class ClosedLoopAeroResult:
    aero: AeroResult
    loads: CoupledAxleLoads
    ride: RideHeightState
    pitch_heave: PitchHeaveState
    breakdown: DeviceBreakdown | None = None


class ClosedLoopAero:
    """
    Stateful aero coupling with optional Phase 9.2 devices.

    devices_enabled=False → identical to Phase 9.1 baseline.
    """

    def __init__(
        self,
        cfg: AeroConfig | None = None,
        ph_params: PitchHeaveParams | None = None,
        mass: float = 1400.0,
        a: float = 1.2,
        b: float = 1.5,
        enable_pitch_dynamics: bool = False,
        device_cfg: AeroDeviceConfig | None = None,
    ):
        self.cfg = cfg or AeroConfig()
        self.ph_params = ph_params or PitchHeaveParams(m=mass, a=a, b=b)
        self.mass = mass
        self.a = a
        self.b = b
        self.enable_pitch_dynamics = enable_pitch_dynamics
        self.ph = PitchHeaveState()
        self.device_cfg = device_cfg or AeroDeviceConfig(devices_enabled=False)
        self.device_solver = AeroDeviceSolver(self.cfg, self.device_cfg)

    def step(
        self,
        inp: VehicleAeroInput,
        ay: float = 0.0,
        dt: float | None = None,
        brake: float = 0.0,
    ) -> ClosedLoopAeroResult:
        dt_use = dt if dt is not None else 0.0

        if self.enable_pitch_dynamics and dt is not None and dt > 0:
            inp_dyn = VehicleAeroInput(
                speed=inp.speed,
                yaw_rate=inp.yaw_rate,
                sideslip=inp.sideslip,
                pitch=self.ph.pitch,
                heave=self.ph.heave,
                yaw_angle=inp.yaw_angle,
                steer=inp.steer,
            )
            ride = ride_from_vehicle_input(inp_dyn, self.cfg)
            aero, breakdown = self._solve_aero(inp.speed, ride, ay=ay, brake=brake, dt=dt_use)
            self._integrate_pitch_heave(aero, dt)
        else:
            ride = ride_from_vehicle_input(inp, self.cfg)
            aero, breakdown = self._solve_aero(inp.speed, ride, ay=ay, brake=brake, dt=dt_use)
            self.ph = PitchHeaveState(pitch=inp.pitch, heave=inp.heave)

        loads = couple_aero_to_tires(
            aero, mass=self.mass, a=self.a, b=self.b, ay=ay
        )
        return ClosedLoopAeroResult(
            aero=aero,
            loads=loads,
            ride=ride,
            pitch_heave=self.ph,
            breakdown=breakdown,
        )

    def _solve_aero(
        self,
        speed: float,
        ride: RideHeightState,
        ay: float,
        brake: float,
        dt: float,
    ) -> tuple[AeroResult, DeviceBreakdown | None]:
        if not self.device_cfg.devices_enabled:
            aero = solve_aero(speed, cfg=self.cfg, ride=ride)
            return aero, None

        dev = self.device_solver.solve(
            speed, ride, ay=ay, brake=brake, dt=dt
        )
        st = dev.state
        aero = AeroResult(
            state=st,
            speed=speed,
            ride=ride,
            config=self.cfg,
            dFz_front=-st.Fz_front,
            dFz_rear=-st.Fz_rear,
            drag_force=st.drag,
            side_force=st.Fy,
            drag_power=st.drag * max(speed, 0.0),
        )
        return aero, dev.breakdown

    def _integrate_pitch_heave(self, aero: AeroResult, dt: float) -> None:
        p = self.ph_params
        z_f = self.ph.heave + p.a * self.ph.pitch
        z_r = self.ph.heave - p.b * self.ph.pitch
        vz_f = self.ph.heave_rate + p.a * self.ph.pitch_rate
        vz_r = self.ph.heave_rate - p.b * self.ph.pitch_rate

        Fs_f = -p.k_front * z_f - p.c_front * vz_f
        Fs_r = -p.k_rear * z_r - p.c_rear * vz_r
        Fa_f = aero.state.Fz_front
        Fa_r = aero.state.Fz_rear
        My_a = aero.state.My

        F_heave = Fs_f + Fs_r + Fa_f + Fa_r
        M_pitch = -p.a * (Fs_f + Fa_f) + p.b * (Fs_r + Fa_r) + My_a

        self.ph.heave_rate += (F_heave / p.m) * dt
        self.ph.pitch_rate += (M_pitch / p.Iy) * dt
        self.ph.heave += self.ph.heave_rate * dt
        self.ph.pitch += self.ph.pitch_rate * dt
        self.ph.pitch = float(np.clip(self.ph.pitch, -0.08, 0.08))
        self.ph.heave = float(np.clip(self.ph.heave, -0.08, 0.08))

    def reset(self) -> None:
        self.ph = PitchHeaveState()
        self.drs_reset()

    def drs_reset(self) -> None:
        self.device_solver.drs.position = 0.0
        self.device_solver.drs._target = 0.0
