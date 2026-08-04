"""
Phase 7.3 – Steady-state Pacejka Magic Formula tire model.

Pure longitudinal and pure lateral MF, combined with a friction-ellipse
clamp for simultaneous slip. Optional camber term on lateral force.

Public API matches DugoffTire:

    longitudinal_lateral_force(slip_ratio, slip_angle, normal_load,
                               camber_rad=0.0) -> TireState

Does not modify Dugoff. Selectable via factory / configuration.
"""

from __future__ import annotations

import numpy as np
from .pacejka_parameters import PacejkaParams, default_passenger_car
from .dugoff import TireState


def _magic_formula(x: float, B: float, C: float, D: float, E: float,
                   Sh: float = 0.0, Sv: float = 0.0) -> float:
    """y = D sin(C arctan(B x_s - E (B x_s - arctan(B x_s)))) + Sv"""
    xs = x + Sh
    Bx = B * xs
    return float(D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx)))) + Sv)


class PacejkaTire:
    """Steady-state Pacejka MF (pure slip + friction clamp)."""

    def __init__(self, params: PacejkaParams | None = None):
        self.p = params or default_passenger_car()

    def longitudinal_force(self, kappa: float, Fz: float) -> float:
        p = self.p
        Fz = max(float(Fz), 1.0)
        kappa = float(np.clip(kappa, -p.kappa_clip, p.kappa_clip))
        Dx = p.mu_x * p.Dx_scale * Fz
        return _magic_formula(kappa, p.Bx, p.Cx, Dx, p.Ex, p.Shx, p.Svx)

    def lateral_force(self, alpha: float, Fz: float, camber_rad: float = 0.0) -> float:
        p = self.p
        Fz = max(float(Fz), 1.0)
        alpha = float(np.clip(alpha, -p.alpha_clip, p.alpha_clip))
        Dy = p.mu_y * p.Dy_scale * Fz
        Fy = _magic_formula(alpha, p.By, p.Cy, Dy, p.Ey, p.Shy, p.Svy)
        # Simple camber thrust (diagnostic-compatible with Phase 7.0/7.1)
        if abs(camber_rad) > 1e-12 and abs(p.C_gamma) > 0.0:
            Fy += p.C_gamma * float(camber_rad) * (Fz / p.Fz0)
        return float(Fy)

    def longitudinal_lateral_force(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        p = self.p
        Fz = max(float(normal_load), 1.0)
        kappa = float(np.clip(slip_ratio, -p.kappa_clip, p.kappa_clip))
        alpha = float(np.clip(slip_angle, -p.alpha_clip, p.alpha_clip))

        # Pure-slip forces (unsaturated MF curves)
        Fx0 = self.longitudinal_force(kappa, Fz)
        Fy0 = self.lateral_force(alpha, Fz, camber_rad=camber_rad)

        Fx, Fy = Fx0, Fy0
        clamp_activated = False
        clamp_scale = 1.0

        # Combined friction limit (ellipse using mu_x / mu_y)
        mu_eff = 0.5 * (p.mu_x + p.mu_y)
        F_max = mu_eff * Fz
        F_mag = float(np.hypot(Fx, Fy))

        if p.enable_friction_clamp and F_mag > F_max + 1e-6:
            clamp_scale = F_max / F_mag
            Fx *= clamp_scale
            Fy *= clamp_scale
            clamp_activated = True

        utilization = float(np.hypot(Fx, Fy) / (F_max + 1e-8))
        saturated = utilization > 0.98 or clamp_activated

        # lambda_ analogue: remaining friction margin (1 = linear region)
        demand = float(np.hypot(Fx0, Fy0)) + 1e-8
        lambda_ = float(min(1.0, F_max / (2.0 * demand)))

        return TireState(
            Fx=float(Fx),
            Fy=float(Fy),
            Fx_linear=float(Fx0),
            Fy_linear=float(Fy0),
            slip_ratio=kappa,
            slip_angle=alpha,
            lambda_=lambda_,
            utilization=utilization,
            saturated=saturated,
            clamp_activated=clamp_activated,
            clamp_scale=float(clamp_scale),
        )
