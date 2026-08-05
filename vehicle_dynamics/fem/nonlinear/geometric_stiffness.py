"""Geometric (stress) stiffness from chord axial force."""

from __future__ import annotations

import numpy as np
from vehicle_dynamics.fem.beam import BeamElement
from .corotational_beam import (
    current_geometry,
    rotation_from_chord,
    axial_force,
    undeformed_length,
)


def local_geometric_stiffness(N: float, L: float) -> np.ndarray:
    if L < 1e-14:
        return np.zeros((12, 12))
    kg = np.zeros((12, 12))
    c = N / L

    kg[1, 1] = 6.0 / 5.0 * c
    kg[1, 7] = -6.0 / 5.0 * c
    kg[7, 1] = -6.0 / 5.0 * c
    kg[7, 7] = 6.0 / 5.0 * c
    kg[1, 5] = c * L / 10.0
    kg[1, 11] = -c * L / 10.0
    kg[5, 1] = c * L / 10.0
    kg[11, 1] = -c * L / 10.0
    kg[7, 5] = -c * L / 10.0
    kg[7, 11] = c * L / 10.0
    kg[5, 7] = -c * L / 10.0
    kg[11, 7] = c * L / 10.0
    kg[5, 5] = 2.0 * c * L**2 / 15.0
    kg[5, 11] = -c * L**2 / 30.0
    kg[11, 5] = -c * L**2 / 30.0
    kg[11, 11] = 2.0 * c * L**2 / 15.0

    kg[2, 2] = 6.0 / 5.0 * c
    kg[2, 8] = -6.0 / 5.0 * c
    kg[8, 2] = -6.0 / 5.0 * c
    kg[8, 8] = 6.0 / 5.0 * c
    kg[2, 4] = -c * L / 10.0
    kg[2, 10] = c * L / 10.0
    kg[4, 2] = -c * L / 10.0
    kg[10, 2] = c * L / 10.0
    kg[8, 4] = c * L / 10.0
    kg[8, 10] = -c * L / 10.0
    kg[4, 8] = c * L / 10.0
    kg[10, 8] = -c * L / 10.0
    kg[4, 4] = 2.0 * c * L**2 / 15.0
    kg[4, 10] = -c * L**2 / 30.0
    kg[10, 4] = -c * L**2 / 30.0
    kg[10, 10] = 2.0 * c * L**2 / 15.0

    return 0.5 * (kg + kg.T)


def geometric_stiffness_global(elem: BeamElement, u: np.ndarray) -> np.ndarray:
    N = axial_force(elem, u)
    L0 = undeformed_length(elem)
    L, pi, pj = current_geometry(elem, u)
    R = rotation_from_chord(pi, pj)
    kg_loc = local_geometric_stiffness(N, L0)
    T = np.zeros((12, 12))
    for i in range(4):
        T[3 * i : 3 * i + 3, 3 * i : 3 * i + 3] = R
    return T.T @ kg_loc @ T
