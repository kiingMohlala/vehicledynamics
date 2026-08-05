"""Aggregate device loads and merge with baseline body aero."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .coefficients import AeroConfig
from .ride_height import RideHeightState
from .aero_model import AeroState, compute_aero_loads, dynamic_pressure
from .aero_devices import AeroDeviceConfig
from .wing_model import evaluate_wing
from .diffuser_model import evaluate_diffuser, evaluate_splitter
from .drs import DRSController
from .active_aero import ActiveAeroController, ActiveAeroMode, ActiveAeroCommand


@dataclass
class DeviceBreakdown:
    front_wing_Fz: float = 0.0
    front_wing_Fx: float = 0.0
    rear_wing_Fz: float = 0.0
    rear_wing_Fx: float = 0.0
    diffuser_Fz: float = 0.0
    diffuser_Fx: float = 0.0
    splitter_Fz: float = 0.0
    splitter_Fx: float = 0.0
    body_Fz_f: float = 0.0
    body_Fz_r: float = 0.0
    body_Fx: float = 0.0
    drs_position: float = 0.0
    rear_wing_alpha: float = 0.0
    active_mode: str = "disabled"
    diffuser_stalled: bool = False
    rear_wing_stalled: bool = False


@dataclass
class DeviceAeroResult:
    state: AeroState
    breakdown: DeviceBreakdown
    command: ActiveAeroCommand | None = None


class AeroDeviceSolver:
    """
    Combines body baseline aero + discrete devices.

    Total Fz_front = body_f + front_wing + splitter
    Total Fz_rear  = body_r + rear_wing + diffuser
    Total Fx       = sum of all drag contributions
    """

    def __init__(
        self,
        aero_cfg: AeroConfig | None = None,
        device_cfg: AeroDeviceConfig | None = None,
    ):
        self.aero_cfg = aero_cfg or AeroConfig()
        self.device_cfg = device_cfg or AeroDeviceConfig()
        self.drs = DRSController(self.device_cfg.drs)
        self.active = ActiveAeroController(self.device_cfg.active)

    def solve(
        self,
        speed: float,
        ride: RideHeightState | None = None,
        *,
        ay: float = 0.0,
        brake: float = 0.0,
        dt: float = 0.0,
        body_scale: float = 1.0,  # additive devices on full body; tune down if double-counting
    ) -> DeviceAeroResult:
        """
        body_scale: fraction of baseline body aero retained (1.0 = full body + additive devices).
        """
        cfg = self.aero_cfg
        dcfg = self.device_cfg
        ride = ride or RideHeightState(h_front=cfg.h_front_ref, h_rear=cfg.h_rear_ref)

        if not cfg.enabled:
            return DeviceAeroResult(state=AeroState(), breakdown=DeviceBreakdown())

        # Baseline body (Phase 9.0/9.1)
        body = compute_aero_loads(speed, cfg, ride=ride)
        q = body.q if speed > 0 else 0.0

        bd = DeviceBreakdown()
        cmd = None

        if not dcfg.devices_enabled or speed <= 0:
            return DeviceAeroResult(state=body, breakdown=bd)

        # Active aero → rear wing angle + DRS command
        rear_alpha = dcfg.rear_wing_alpha
        if dcfg.use_active_aero:
            cmd = self.active.update(speed, ay=ay, brake=brake)
            rear_alpha = cmd.rear_wing_alpha
            if dcfg.use_drs:
                self.drs.command(cmd.drs_open)
            bd.active_mode = cmd.mode.value
        else:
            bd.active_mode = ActiveAeroMode.DISABLED.value
            if dcfg.use_drs:
                self.drs.command(False)

        if dt > 0 and dcfg.use_drs:
            self.drs.step(dt)
        bd.drs_position = self.drs.position
        bd.rear_wing_alpha = rear_alpha
        cl_drs, cd_drs = self.drs.factors() if dcfg.use_drs else (1.0, 1.0)

        # Body reduced share
        scale = body_scale
        Fz_f = body.Fz_front * scale
        Fz_r = body.Fz_rear * scale
        Fx = body.Fx * scale
        bd.body_Fz_f = Fz_f
        bd.body_Fz_r = Fz_r
        bd.body_Fx = Fx

        if dcfg.use_front_wing:
            fw = evaluate_wing(q, dcfg.front_wing_alpha, dcfg.front_wing)
            Fz_f += fw.Fz
            Fx += fw.Fx
            bd.front_wing_Fz = fw.Fz
            bd.front_wing_Fx = fw.Fx

        if dcfg.use_splitter:
            sp = evaluate_splitter(q, ride.h_front, dcfg.splitter)
            Fz_f += sp.Fz
            Fx += sp.Fx
            bd.splitter_Fz = sp.Fz
            bd.splitter_Fx = sp.Fx

        if dcfg.use_rear_wing:
            rw = evaluate_wing(q, rear_alpha, dcfg.rear_wing)
            # Apply DRS factors to rear wing only
            rw_Fz = rw.Fz * cl_drs
            rw_Fx = rw.Fx * cd_drs  # Fx already negative; scale magnitude via Cd factor
            # Cd factor: open DRS reduces |Fx|
            if dcfg.use_drs:
                rw_Fx = -abs(rw.Fx) * cd_drs
                rw_Fz = -abs(rw.Fz) * cl_drs  # keep downforce sign
            Fz_r += rw_Fz
            Fx += rw_Fx
            bd.rear_wing_Fz = rw_Fz
            bd.rear_wing_Fx = rw_Fx
            bd.rear_wing_stalled = rw.stalled

        if dcfg.use_diffuser:
            diff = evaluate_diffuser(q, ride.h_rear, ride.rake, dcfg.diffuser)
            Fz_r += diff.Fz
            Fx += diff.Fx
            bd.diffuser_Fz = diff.Fz
            bd.diffuser_Fx = diff.Fx
            bd.diffuser_stalled = diff.stalled

        # Moments / CoP from axle forces
        L = cfg.wheelbase
        a = 0.5 * L
        b = 0.5 * L
        My = -Fz_f * a + Fz_r * b
        Fz_tot = Fz_f + Fz_r
        x_cp = (Fz_r * b - Fz_f * a) / Fz_tot if abs(Fz_tot) > 1e-9 else 0.0

        S = cfg.frontal_area
        Cd_eff = -Fx / (q * S) if q * S > 1e-9 else 0.0
        Cl_f_eff = Fz_f / (q * S) if q * S > 1e-9 else 0.0
        Cl_r_eff = Fz_r / (q * S) if q * S > 1e-9 else 0.0

        state = AeroState(
            q=q,
            Fx=Fx,
            Fy=body.Fy,
            Fz_front=Fz_f,
            Fz_rear=Fz_r,
            Mx=0.0,
            My=My,
            Mz=body.Mz,
            Cd_eff=Cd_eff,
            Cl_front_eff=Cl_f_eff,
            Cl_rear_eff=Cl_r_eff,
            center_of_pressure_x=x_cp,
            cooling_drag=body.cooling_drag * scale,
        )
        return DeviceAeroResult(state=state, breakdown=bd, command=cmd)
