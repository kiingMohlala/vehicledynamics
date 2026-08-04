"""
Pacejka Magic Formula parameter sets (Phase 7.3).

Simplified MF pure-slip coefficients with load and friction scaling.
Values are representative passenger-car defaults, not fitted tire data.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PacejkaParams:
    """
    Compact Pacejka MF parameters for longitudinal and lateral pure slip.

    Standard form:
        y = D sin(C arctan(B x - E (B x - arctan(B x))))
        F = y + S_v   (with shift S_h on input)

    Scaling:
        D_x, D_y scaled by μ and Fz/Fz0
    """

    # Nominal load [N]
    Fz0: float = 4000.0

    # Friction / peak factors
    mu_x: float = 1.0
    mu_y: float = 1.0

    # Longitudinal (kappa)
    Bx: float = 12.0
    Cx: float = 1.65
    Dx_scale: float = 1.0   # multiplies mu_x * Fz
    Ex: float = 0.10
    Shx: float = 0.0
    Svx: float = 0.0

    # Lateral (alpha in radians)
    By: float = 10.0
    Cy: float = 1.30
    Dy_scale: float = 1.0
    Ey: float = -0.50
    Shy: float = 0.0
    Svy: float = 0.0

    # Camber influence on lateral peak (simple)
    # Fy_camber ≈ C_gamma * gamma * (Fz/Fz0)
    C_gamma: float = 0.0

    # Geometry / numerics
    radius: float = 0.30
    v_eps: float = 0.5
    kappa_clip: float = 1.0
    alpha_clip: float = 1.2  # ~70 deg

    # Combined-slip friction ellipse safety clamp
    enable_friction_clamp: bool = True


def default_passenger_car() -> PacejkaParams:
    return PacejkaParams()


def high_mu_race() -> PacejkaParams:
    return PacejkaParams(mu_x=1.4, mu_y=1.4, Bx=14.0, By=12.0, Dx_scale=1.05, Dy_scale=1.05)


def low_mu_wet() -> PacejkaParams:
    return PacejkaParams(mu_x=0.55, mu_y=0.55, Bx=10.0, By=8.0)
