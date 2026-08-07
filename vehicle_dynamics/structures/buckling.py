"""Euler buckling approximations for beam members."""
from __future__ import annotations

import numpy as np
from .materials import StructuralMaterial, steel


def euler_critical_load(L: float, E: float, I: float, K_factor: float = 1.0) -> float:
    """Pcr = π² EI / (K L)²."""
    Le = K_factor * L
    return (np.pi ** 2) * E * I / (Le ** 2 + 1e-30)


def member_buckling_sf(axial_force: float, L: float, mat: StructuralMaterial | None = None, I: float = 1e-7, K_factor: float = 1.0) -> float:
    mat = mat or steel()
    Pcr = euler_critical_load(L, mat.E, I, K_factor)
    if axial_force >= 0:
        return float("inf")  # tension
    return float(Pcr / (abs(axial_force) + 1e-9))
