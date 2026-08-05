"""Front and rear wing aerodynamic models."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class WingParams:
    area: float = 0.35          # m²
    Cl0: float = 0.9            # at zero AoA (magnitude; sign applied as downforce)
    Cl_alpha: float = 3.5       # per rad
    Cd0: float = 0.05
    induced_factor: float = 0.08  # k in Cd = Cd0 + k Cl²
    stall_alpha: float = 0.25   # rad (~14°)
    efficiency: float = 0.92
    # Downforce is negative Cl in vehicle frame; wings produce positive "Cl_wing" magnitude


@dataclass
class WingState:
    alpha: float
    Cl: float
    Cd: float
    Fz: float   # vertical force on vehicle (neg = downforce)
    Fx: float   # longitudinal (neg = drag)
    stalled: bool


def _stall_factor(alpha: float, alpha_stall: float) -> float:
    a = abs(alpha)
    if a <= alpha_stall:
        return 1.0
    # Soft post-stall decay
    over = (a - alpha_stall) / max(alpha_stall, 1e-6)
    return float(np.clip(1.0 - 0.6 * over, 0.25, 1.0))


def evaluate_wing(
    q: float,
    alpha: float,
    params: WingParams,
    *,
    downforce_sign: float = -1.0,
) -> WingState:
    """
    alpha in rad (geometric angle of attack).
    downforce_sign=-1 → positive alpha produces downforce (Fz < 0).
    """
    sf = _stall_factor(alpha, params.stall_alpha)
    Cl_mag = (params.Cl0 + params.Cl_alpha * abs(alpha)) * params.efficiency * sf
    Cl = downforce_sign * np.sign(alpha + 1e-15) * Cl_mag if alpha != 0 else downforce_sign * Cl_mag
    # Prefer consistent downforce for positive AoA setup
    Cl = downforce_sign * Cl_mag
    Cd = params.Cd0 + params.induced_factor * (Cl_mag ** 2)
    Fz = Cl * q * params.area
    Fx = -Cd * q * params.area
    stalled = abs(alpha) > params.stall_alpha
    return WingState(alpha=alpha, Cl=Cl, Cd=Cd, Fz=Fz, Fx=Fx, stalled=stalled)
