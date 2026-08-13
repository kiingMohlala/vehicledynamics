"""Standalone Euler-Bernoulli beam stiffness (12x12 local) for unit tests / simple frames."""
from __future__ import annotations

import numpy as np
from .materials import StructuralMaterial, steel


def local_beam_stiffness(L: float, mat: StructuralMaterial | None = None, A: float = 5e-4, Iy: float = 1e-7, Iz: float = 1e-7, J: float = 2e-7) -> np.ndarray:
    """12x12 local stiffness [ux,uy,uz,rx,ry,rz] x 2 nodes."""
    mat = mat or steel()
    E, G = mat.E, mat.G
    k = np.zeros((12, 12))
    # axial
    k[0, 0] = k[6, 6] = E * A / L
    k[0, 6] = k[6, 0] = -E * A / L
    # torsion
    k[3, 3] = k[9, 9] = G * J / L
    k[3, 9] = k[9, 3] = -G * J / L
    # bending about y (deflection in z)
    for idx, I in ((1, Iz), (2, Iy)):  # uy uses Iz, uz uses Iy
        # map: uy=1 rz=5 ; uz=2 ry=4
        if idx == 1:
            t, r = 1, 5
            t2, r2 = 7, 11
        else:
            t, r = 2, 4
            t2, r2 = 8, 10
        a = 12 * E * I / L**3
        b = 6 * E * I / L**2
        c = 4 * E * I / L
        d = 2 * E * I / L
        k[t, t] = k[t2, t2] = a
        k[t, t2] = k[t2, t] = -a
        k[t, r] = k[r, t] = b
        k[t, r2] = k[r2, t] = b
        k[t2, r] = k[r, t2] = -b
        k[t2, r2] = k[r2, t2] = -b
        k[r, r] = k[r2, r2] = c
        k[r, r2] = k[r2, r] = d
    return k


def cantilever_tip_deflection(P: float, L: float, E: float, I: float) -> float:
    """Analytical PL^3/(3EI)."""
    return P * L**3 / (3 * E * I)
