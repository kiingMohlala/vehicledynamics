"""
Steady-state Pacejka Magic Formula with combined-slip weighting (Phase 7.6).

Flow:
    pure MF Fx/Fy  →  Gx(α), Gy(κ) reduction  →  safety clamp  →  TireState

combined_slip=False → identical to Phase 7.5 (pure MF + clamp only).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .pacejka_parameters import PacejkaParams, default_passenger_car
from .load_sensitivity import effective_mu


@dataclass
class TireState:
    Fx: float
    Fy: float
    Fx_linear: float
    Fy_linear: float
    slip_ratio: float
    slip_angle: float
    lambda_: float
    utilization: float
    saturated: bool
    clamp_activated: bool = False
    clamp_scale: float = 1.0
    # Phase 7.6 diagnostics (optional)
    combined_Gx: float = 1.0
    combined_Gy: float = 1.0
    Fx_pure: float = 0.0
    Fy_pure: float = 0.0


def _magic_formula(
    x: float, B: float, C: float, D: float, E: float, Sh: float = 0.0, Sv: float = 0.0
) -> float:
    xs = x + Sh
    Bx = B * xs
    return float(D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx)))) + Sv)


def combined_weight_x(alpha: float, alpha_c: float) -> float:
    """Gx(α) ≤ 1 — lateral slip reduces available longitudinal force."""
    ac = max(float(alpha_c), 1e-6)
    return float(1.0 / np.sqrt(1.0 + (float(alpha) / ac) ** 2))


def combined_weight_y(kappa: float, kappa_c: float) -> float:
    """Gy(κ) ≤ 1 — longitudinal slip reduces available lateral force."""
    kc = max(float(kappa_c), 1e-6)
    return float(1.0 / np.sqrt(1.0 + (float(kappa) / kc) ** 2))


class PacejkaTire:
    def __init__(self, params: PacejkaParams | None = None):
        self.p = params or default_passenger_car()

    def _mu_x(self, Fz: float) -> float:
        if not self.p.load_sensitive:
            return float(self.p.mu_x)
        return effective_mu(self.p.mu_x, Fz, self.p.Fz0, self.p.load_exponent)

    def _mu_y(self, Fz: float) -> float:
        if not self.p.load_sensitive:
            return float(self.p.mu_y)
        return effective_mu(self.p.mu_y, Fz, self.p.Fz0, self.p.load_exponent)

    def longitudinal_force(self, kappa: float, Fz: float) -> float:
        p = self.p
        Fz = max(float(Fz), 1.0)
        kappa = float(np.clip(kappa, -p.kappa_clip, p.kappa_clip))
        Dx = self._mu_x(Fz) * p.Dx_scale * Fz
        return _magic_formula(kappa, p.Bx, p.Cx, Dx, p.Ex, p.Shx, p.Svx)

    def lateral_force(
        self, alpha: float, Fz: float, camber_rad: float = 0.0
    ) -> float:
        p = self.p
        Fz = max(float(Fz), 1.0)
        alpha = float(np.clip(alpha, -p.alpha_clip, p.alpha_clip))
        Dy = self._mu_y(Fz) * p.Dy_scale * Fz
        Fy = _magic_formula(alpha, p.By, p.Cy, Dy, p.Ey, p.Shy, p.Svy)
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

        Fx_pure = self.longitudinal_force(kappa, Fz)
        Fy_pure = self.lateral_force(alpha, Fz, camber_rad=camber_rad)

        if p.combined_slip:
            Gx = combined_weight_x(alpha, p.alpha_combined)
            Gy = combined_weight_y(kappa, p.kappa_combined)
        else:
            Gx, Gy = 1.0, 1.0

        Fx = Fx_pure * Gx
        Fy = Fy_pure * Gy

        clamp_activated = False
        clamp_scale = 1.0

        mu_eff = 0.5 * (self._mu_x(Fz) + self._mu_y(Fz))
        F_max = mu_eff * Fz
        F_mag = float(np.hypot(Fx, Fy))

        if p.enable_friction_clamp and F_mag > F_max + 1e-6:
            clamp_scale = F_max / F_mag
            Fx *= clamp_scale
            Fy *= clamp_scale
            clamp_activated = True

        utilization = float(np.hypot(Fx, Fy) / (F_max + 1e-8))
        saturated = utilization > 0.98 or clamp_activated

        demand = float(np.hypot(Fx_pure, Fy_pure)) + 1e-8
        lambda_ = float(min(1.0, F_max / (2.0 * demand)))

        return TireState(
            Fx=float(Fx),
            Fy=float(Fy),
            Fx_linear=float(Fx_pure),
            Fy_linear=float(Fy_pure),
            slip_ratio=kappa,
            slip_angle=alpha,
            lambda_=lambda_,
            utilization=utilization,
            saturated=saturated,
            clamp_activated=clamp_activated,
            clamp_scale=float(clamp_scale),
            combined_Gx=float(Gx),
            combined_Gy=float(Gy),
            Fx_pure=float(Fx_pure),
            Fy_pure=float(Fy_pure),
        )
