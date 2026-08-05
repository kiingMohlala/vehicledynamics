"""Core aerodynamic force/moment computation."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .coefficients import AeroConfig, AeroCoefficients
from .ride_height import RideHeightState, ride_height_factors


@dataclass
class AeroState:
    """Instantaneous aero loads in vehicle frame.

    +x forward, +z upward, +y to the left.
    Downforce is negative Fz (or reported as positive downforce magnitude).
    """

    q: float = 0.0              # dynamic pressure Pa
    Fx: float = 0.0             # drag (negative when resisting forward motion)
    Fy: float = 0.0             # side force
    Fz_front: float = 0.0       # vertical force on front axle (neg = down)
    Fz_rear: float = 0.0
    Mx: float = 0.0             # roll
    My: float = 0.0             # pitch
    Mz: float = 0.0             # yaw
    Cd_eff: float = 0.0
    Cl_front_eff: float = 0.0
    Cl_rear_eff: float = 0.0
    center_of_pressure_x: float = 0.0  # from CG, + aft of CG
    cooling_drag: float = 0.0

    @property
    def drag(self) -> float:
        return -self.Fx  # positive drag magnitude

    @property
    def downforce_front(self) -> float:
        return -self.Fz_front

    @property
    def downforce_rear(self) -> float:
        return -self.Fz_rear

    @property
    def downforce_total(self) -> float:
        return self.downforce_front + self.downforce_rear

    @property
    def L_over_D(self) -> float:
        d = self.drag
        if d < 1e-9:
            return 0.0
        return self.downforce_total / d

    @property
    def front_balance(self) -> float:
        total = self.downforce_total
        if total < 1e-9:
            return 0.5
        return self.downforce_front / total


def dynamic_pressure(rho: float, speed: float) -> float:
    return 0.5 * rho * speed * speed


def compute_aero_loads(
    speed: float,
    cfg: AeroConfig,
    ride: RideHeightState | None = None,
    coeffs: AeroCoefficients | None = None,
) -> AeroState:
    """
    Steady-state aero loads at vehicle speed [m/s].

    If cfg.enabled is False, returns zeros (regression contract).
    """
    if not cfg.enabled or speed <= 0.0:
        return AeroState()

    ride = ride or RideHeightState(
        h_front=cfg.h_front_ref, h_rear=cfg.h_rear_ref
    )
    base = coeffs or cfg.coeffs
    fac = ride_height_factors(ride, cfg)

    Cd = base.Cd * fac["Cd"]
    Cl_f = base.Cl_front * fac["Cl_front"]
    Cl_r = base.Cl_rear * fac["Cl_rear"]
    Cy = base.Cy_beta * fac["Cy"] * ride.yaw_rad

    q = dynamic_pressure(cfg.rho, speed)
    S = cfg.frontal_area
    L = cfg.wheelbase
    c = cfg.ref_chord

    # Forces: Cd > 0 → drag opposes +x motion → Fx negative
    cooling = cfg.cooling_drag_fraction * Cd * q * S
    Fx = -(Cd * q * S)
    Fy = Cy * q * S  # Cy already includes yaw
    Fz_f = Cl_f * q * S   # Cl_f < 0 → Fz_f < 0 (down)
    Fz_r = Cl_r * q * S

    # Pitch moment about CG from axle force imbalance
    # Front axle at +a from CG? Use balance about mid-wheelbase approx
    a = 0.5 * L  # CG at mid for Phase 9.0 default
    b = 0.5 * L
    My = -Fz_f * a + Fz_r * b + base.Cm_pitch * q * S * c

    Mz = base.Cn_yaw * ride.yaw_rad * q * S * L
    Mx = 0.0

    # Center of pressure (longitudinal from vehicle mid): where total Fz acts
    Fz_tot = Fz_f + Fz_r
    if abs(Fz_tot) > 1e-9:
        # x_cp from mid: (Fz_r * b - Fz_f * a) / Fz_tot — positive aft
        x_cp = (Fz_r * b - Fz_f * a) / Fz_tot
    else:
        x_cp = 0.0

    return AeroState(
        q=q,
        Fx=Fx,
        Fy=Fy,
        Fz_front=Fz_f,
        Fz_rear=Fz_r,
        Mx=Mx,
        My=My,
        Mz=Mz,
        Cd_eff=Cd,
        Cl_front_eff=Cl_f,
        Cl_rear_eff=Cl_r,
        center_of_pressure_x=x_cp,
        cooling_drag=cooling,
    )
