from dataclasses import dataclass
import numpy as np

@dataclass
class DugoffParams:
    mu: float = 1.0
    Cx: float = 80000.0
    Cy: float = 80000.0
    radius: float = 0.30
    v_eps: float = 0.5

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
    def __init__(self, params: DugoffParams = None):
        self.p = params or DugoffParams()

    def longitudinal_lateral_force(self, slip_ratio: float, slip_angle: float, normal_load: float) -> TireState:
        kappa = np.clip(slip_ratio, -1.0, 1.0)
        alpha = np.clip(slip_angle, -np.pi/2, np.pi/2)
        Fz = max(normal_load, 1.0)

        Fx0 = self.p.Cx * kappa / (1.0 + abs(kappa))
        Fy0 = self.p.Cy * alpha

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

        F_mag = np.sqrt(Fx**2 + Fy**2)
        F_max = self.p.mu * Fz
        clamp_activated = False
        clamp_scale = 1.0

        if F_mag > F_max + 1e-6:
            clamp_scale = F_max / F_mag
            Fx *= clamp_scale
            Fy *= clamp_scale
            clamp_activated = True

        utilization = np.sqrt(Fx**2 + Fy**2) / (self.p.mu * Fz + 1e-8)

        return TireState(
            Fx=Fx, Fy=Fy, Fx_linear=Fx0, Fy_linear=Fy0,
            slip_ratio=kappa, slip_angle=alpha,
            lambda_=lambda_, utilization=utilization, saturated=saturated,
            clamp_activated=clamp_activated, clamp_scale=clamp_scale
        )
