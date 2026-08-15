"""
Phase 14.3 — Relative airflow aerodynamic coupling.

Vehicle velocity + wind velocity → relative air velocity → β → Cy/Cn → Fy/Mz.

Replaces the external crosswind disturbance (st.crosswind * 40 N) with a
physically coupled aero model driven by relative airflow.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .coefficients import AeroConfig, AeroCoefficients
from .aero_model import AeroState, dynamic_pressure, compute_aero_loads
from .ride_height import RideHeightState


@dataclass
class RelativeAirflowState:
    """Relative airflow diagnostics and aero loads in vehicle body frame."""

    wind_vx: float = 0.0          # wind in body frame, +x forward (m/s)
    wind_vy: float = 0.0          # wind in body frame, +y left (m/s)
    rel_air_vx: float = 0.0       # V_air_x = vx - wind_vx
    rel_air_vy: float = 0.0
    relative_air_speed: float = 0.0
    beta_air: float = 0.0         # rad, atan2(rel_vy, rel_vx)
    alpha_air: float = 0.0        # reserved (pitch); 0 in planar model
    Fx_aero: float = 0.0          # body +x (drag is negative when resisting)
    Fy_aero: float = 0.0          # body +y
    Mz_aero: float = 0.0          # yaw moment
    Fz_front: float = 0.0
    Fz_rear: float = 0.0
    Cd: float = 0.0
    Cl_front: float = 0.0
    Cl_rear: float = 0.0
    Cy: float = 0.0               # Cy_beta * beta
    Cn: float = 0.0               # Cn_beta * beta
    q: float = 0.0
    drag: float = 0.0             # positive magnitude opposing forward motion
    downforce_total: float = 0.0


def body_wind_from_world(
    wind_wx: float,
    wind_wy: float,
    psi: float,
) -> tuple[float, float]:
    """Transform world-frame wind into vehicle body frame."""
    c, s = np.cos(psi), np.sin(psi)
    # wind_body = R(-psi) * wind_world
    bx = wind_wx * c + wind_wy * s
    by = -wind_wx * s + wind_wy * c
    return float(bx), float(by)


def relative_air_velocity(
    vx: float,
    vy: float,
    wind_vx: float,
    wind_vy: float,
) -> tuple[float, float, float, float]:
    """
    V_air = V_vehicle - V_wind  (body frame).

    Returns (rel_vx, rel_vy, air_speed, beta_air).
    beta = 0 when relative flow is pure +x (no sideslip relative to air).
    """
    rel_vx = float(vx - wind_vx)
    rel_vy = float(vy - wind_vy)
    air_speed = float(np.hypot(rel_vx, rel_vy))
    if air_speed < 1e-6:
        beta = 0.0
    else:
        beta = float(np.arctan2(rel_vy, rel_vx))
    return rel_vx, rel_vy, air_speed, beta


def compute_sideslip_aero(
    vx: float,
    vy: float,
    wind_vx: float,
    wind_vy: float,
    cfg: AeroConfig,
    ride: RideHeightState | None = None,
    *,
    Cy_beta: float | None = None,
    Cn_beta: float | None = None,
    draft_factor: float = 1.0,
) -> RelativeAirflowState:
    """
    Full relative-airflow aero state.

    Longitudinal/downforce from relative air speed (not vehicle speed alone).
    Side force / yaw moment from β_air via linear Cyβ, Cnβ.
    """
    out = RelativeAirflowState(wind_vx=wind_vx, wind_vy=wind_vy)
    if not cfg.enabled:
        return out

    rel_vx, rel_vy, air_speed, beta = relative_air_velocity(vx, vy, wind_vx, wind_vy)
    out.rel_air_vx = rel_vx
    out.rel_air_vy = rel_vy
    out.relative_air_speed = air_speed
    out.beta_air = beta

    if air_speed < 0.05:
        return out

    base = cfg.coeffs
    cy_b = float(Cy_beta if Cy_beta is not None else base.Cy_beta)
    cn_b = float(Cn_beta if Cn_beta is not None else base.Cn_yaw)

    # Longitudinal + downforce from relative air speed
    aero = compute_aero_loads(air_speed, cfg, ride=ride)
    # Override side/yaw with explicit β model (compute_aero_loads uses ride.yaw_rad)
    q = dynamic_pressure(cfg.rho, air_speed)
    S = cfg.frontal_area
    L = cfg.wheelbase

    Cy = cy_b * beta
    Cn = cn_b * beta
    Fy = Cy * q * S
    Mz = Cn * q * S * L

    out.q = q
    out.Fx_aero = float(aero.Fx)  # already negative for drag
    out.Fy_aero = float(Fy)
    out.Mz_aero = float(Mz)
    out.Fz_front = float(aero.Fz_front)
    out.Fz_rear = float(aero.Fz_rear)
    out.Cd = float(aero.Cd_eff)
    out.Cl_front = float(aero.Cl_front_eff)
    out.Cl_rear = float(aero.Cl_rear_eff)
    out.Cy = float(Cy)
    out.Cn = float(Cn)
    out.drag = float(aero.drag * draft_factor)
    out.downforce_total = float(aero.downforce_total)
    return out
