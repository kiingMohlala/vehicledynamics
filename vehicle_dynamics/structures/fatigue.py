"""Basic S-N fatigue estimation (Basquin-like)."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .materials import StructuralMaterial, steel


@dataclass
class FatigueResult:
    damage: float
    life_cycles: float
    safe: bool


def basquin_life(sigma_a: float, mat: StructuralMaterial | None = None, Sf: float | None = None, b: float = -0.12) -> float:
    """
    sigma_a = Sf * (2 N)^b  => N from stress amplitude.
    Very approximate.
    """
    mat = mat or steel()
    Sf = Sf if Sf is not None else (mat.fatigue_endurance or 0.5 * mat.Su)
    if sigma_a < 1e-3:
        return 1e12
    # sigma_a = Sf * (2N)^b  => 2N = (sigma_a/Sf)^(1/b)
    ratio = sigma_a / max(Sf, 1.0)
    if ratio <= 1e-6:
        return 1e12
    twoN = ratio ** (1.0 / b)
    return float(max(twoN / 2.0, 1.0))


def miner_damage(stress_amplitudes: list[float], cycles: list[float], mat: StructuralMaterial | None = None) -> FatigueResult:
    mat = mat or steel()
    D = 0.0
    min_life = 1e12
    for sa, n in zip(stress_amplitudes, cycles):
        N = basquin_life(sa, mat)
        min_life = min(min_life, N)
        D += n / N
    return FatigueResult(damage=float(D), life_cycles=float(min_life), safe=D < 1.0)
