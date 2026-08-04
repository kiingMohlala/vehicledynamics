"""
Combined-Slip Dugoff Tire Model (Phase 3.4 + Phase 7.0 camber)

Coordinate system:
  +x : forward
  +y : left
  +z : upward

Sign conventions:
  Fx > 0  → force in +x direction (forward traction)
  Fy > 0  → force in +y direction (to the left)
  κ  > 0  → braking slip
  α  > 0  → sideslip to the left
  γ  > 0  → camber (top of tire toward +y)

Slip definitions:
  κ = (Vx - ωR) / max(|Vx|, v_eps)
  α = atan2(Vy, max(|Vx|, v_eps))

Phase 7.0 – Camber thrust (optional):
  Fy_γ = Cγ · (Fz / Fz_ref) · γ
  Applied after combined-slip scaling of (Fx0, Fy0), then friction clamp.
  camber_rad = 0 → identical to Phase 3.4 Dugoff (regression).
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class DugoffParams:
    mu: float = 1.0
    Cx: float = 80000.0      # Longitudinal stiffness [N]
    Cy: float = 80000.0      # Lateral stiffness [N/rad]
    radius: float = 0.30
    v_eps: float = 0.5
    # Phase 7.0 camber
    C_gamma: float = 2000.0  # Camber thrust stiffness [N/rad] at Fz_ref
    Fz_ref: float = 4000.0   # Reference load for camber thrust scaling [N]
    camber_enabled: bool = True


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
    # Phase 7.0
    camber_rad: float = 0.0
    Fy_camber: float = 0.0


class DugoffTire:
    def __init__(self, params: DugoffParams = None):
        self.p = params or DugoffParams()

    def longitudinal_lateral_force(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        kappa = np.clip(slip_ratio, -1.0, 1.0)
        alpha = np.clip(slip_angle, -np.pi / 2, np.pi / 2)
        gamma = float(camber_rad)
        Fz = max(normal_load, 1.0)

        # Linear (unsaturated) forces – longitudinal form preserved from Phase 3.3
        Fx0 = self.p.Cx * kappa / (1.0 + abs(kappa))
        Fy0 = self.p.Cy * alpha  # small-angle approximation

        # Combined-slip saturation factor (slip forces only)
        F_demand = np.sqrt(Fx0**2 + Fy0**2) + 1e-8
        lambda_ = (self.p.mu * Fz) / (2.0 * F_demand)

        if lambda_ >= 1.0:
            f = 1.0
            saturated = False
        else:
            f = lambda_ * (2.0 - lambda_)
            saturated = True

        Fx = Fx0 * f
        Fy = Fy0 * f

        # Phase 7.0 – camber thrust (load-scaled), then shared friction limit
        Fy_camber = 0.0
        if self.p.camber_enabled and abs(gamma) > 1e-12:
            Fy_camber = self.p.C_gamma * (Fz / max(self.p.Fz_ref, 1.0)) * gamma
            Fy = Fy + Fy_camber

        # Safety clamp on total force
        F_mag = np.sqrt(Fx**2 + Fy**2)
        F_max = self.p.mu * Fz
        clamp_activated = False
        clamp_scale = 1.0

        if F_mag > F_max + 1e-6:
            clamp_scale = F_max / F_mag
            Fx *= clamp_scale
            Fy *= clamp_scale
            Fy_camber *= clamp_scale
            clamp_activated = True

        utilization = np.sqrt(Fx**2 + Fy**2) / (self.p.mu * Fz + 1e-8)

        return TireState(
            Fx=Fx,
            Fy=Fy,
            Fx_linear=Fx0,
            Fy_linear=Fy0,
            slip_ratio=kappa,
            slip_angle=alpha,
            lambda_=lambda_,
            utilization=utilization,
            saturated=saturated,
            clamp_activated=clamp_activated,
            clamp_scale=clamp_scale,
            camber_rad=gamma,
            Fy_camber=Fy_camber,
        )
