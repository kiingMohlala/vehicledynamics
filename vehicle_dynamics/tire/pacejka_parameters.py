"""Pacejka Magic Formula parameter sets (Phases 7.3–7.6)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PacejkaParams:
    Fz0: float = 4000.0
    mu_x: float = 1.0
    mu_y: float = 1.0

    Bx: float = 12.0
    Cx: float = 1.65
    Dx_scale: float = 1.0
    Ex: float = 0.10
    Shx: float = 0.0
    Svx: float = 0.0

    By: float = 10.0
    Cy: float = 1.30
    Dy_scale: float = 1.0
    Ey: float = -0.50
    Shy: float = 0.0
    Svy: float = 0.0

    C_gamma: float = 0.0

    radius: float = 0.30
    v_eps: float = 0.5
    kappa_clip: float = 1.0
    alpha_clip: float = 1.2

    enable_friction_clamp: bool = True

    # Phase 7.5
    load_sensitive: bool = False
    load_exponent: float = 0.08

    # Phase 7.6 – combined-slip weighting
    combined_slip: bool = True
    alpha_combined: float = 0.15   # α_c [rad]
    kappa_combined: float = 0.12   # κ_c [-]


def default_passenger_car() -> PacejkaParams:
    return PacejkaParams()


def high_mu_race() -> PacejkaParams:
    return PacejkaParams(
        mu_x=1.4, mu_y=1.4, Bx=14.0, By=12.0, Dx_scale=1.05, Dy_scale=1.05
    )


def low_mu_wet() -> PacejkaParams:
    return PacejkaParams(mu_x=0.55, mu_y=0.55, Bx=10.0, By=8.0)
