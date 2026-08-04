"""
Dugoff tire model with optional load-sensitive friction (Phase 7.5).

load_sensitive=False → identical to Phase 7.4 baseline.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .load_sensitivity import effective_mu


@dataclass
class DugoffParams:
    mu: float = 1.0
    Cx: float = 80000.0
    Cy: float = 80000.0
    radius: float = 0.30
    v_eps: float = 0.5
    # Phase 7.5
    load_sensitive: bool = False
    Fz0: float = 4000.0
    load_exponent: float = 0.08


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


class DugoffTire:
    def __init__(self, params: DugoffParams | None = None):
        self.p = params or DugoffParams()

    def _mu(self, Fz: float) -> float:
        if not self.p.load_sensitive:
            return float(self.p.mu)
        return effective_mu(self.p.mu, Fz, self.p.Fz0, self.p.load_exponent)

    def longitudinal_lateral_force(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        # camber_rad accepted for API compatibility; Dugoff pure form ignores it
        # (camber thrust is handled by dedicated variants / Pacejka C_gamma)
        _ = camber_rad
        kappa = float(np.clip(slip_ratio, -1.0, 1.0))
        alpha = float(np.clip(slip_angle, -np.pi / 2, np.pi / 2))
        Fz = max(float(normal_load), 1.0)
        mu = self._mu(Fz)

        Fx0 = self.p.Cx * kappa / (1.0 + abs(kappa))
        Fy0 = self.p.Cy * alpha

        F_demand = float(np.hypot(Fx0, Fy0)) + 1e-8
        lambda_ = (mu * Fz) / (2.0 * F_demand)

        if lambda_ >= 1.0:
            f = 1.0
            saturated = False
        else:
            f = lambda_ * (2.0 - lambda_)
            saturated = True

        Fx = Fx0 * f
        Fy = Fy0 * f

        F_mag = float(np.hypot(Fx, Fy))
        F_max = mu * Fz
        clamp_activated = False
        clamp_scale = 1.0

        if F_mag > F_max + 1e-6:
            clamp_scale = F_max / F_mag
            Fx *= clamp_scale
            Fy *= clamp_scale
            clamp_activated = True

        utilization = float(np.hypot(Fx, Fy) / (mu * Fz + 1e-8))

        return TireState(
            Fx=float(Fx),
            Fy=float(Fy),
            Fx_linear=float(Fx0),
            Fy_linear=float(Fy0),
            slip_ratio=kappa,
            slip_angle=alpha,
            lambda_=float(lambda_),
            utilization=utilization,
            saturated=saturated,
            clamp_activated=clamp_activated,
            clamp_scale=float(clamp_scale),
        )
