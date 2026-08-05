"""Euler-Bernoulli beam local stiffness (12×12)."""

from __future__ import annotations

import numpy as np
from .beam import BeamElement
from .transform import transformation_matrix


def local_stiffness(elem: BeamElement) -> np.ndarray:
    L = elem.length()
    E = elem.material.E
    G = elem.material.G
    A = elem.section.A
    Iy = elem.section.Iy
    Iz = elem.section.Iz
    J = elem.section.J

    k = np.zeros((12, 12))

    ea_l = E * A / L
    k[0, 0] = ea_l
    k[0, 6] = -ea_l
    k[6, 0] = -ea_l
    k[6, 6] = ea_l

    gj_l = G * J / L
    k[3, 3] = gj_l
    k[3, 9] = -gj_l
    k[9, 3] = -gj_l
    k[9, 9] = gj_l

    eiz = E * Iz
    k[1, 1] = 12 * eiz / L**3
    k[1, 5] = 6 * eiz / L**2
    k[1, 7] = -12 * eiz / L**3
    k[1, 11] = 6 * eiz / L**2
    k[5, 1] = 6 * eiz / L**2
    k[5, 5] = 4 * eiz / L
    k[5, 7] = -6 * eiz / L**2
    k[5, 11] = 2 * eiz / L
    k[7, 1] = -12 * eiz / L**3
    k[7, 5] = -6 * eiz / L**2
    k[7, 7] = 12 * eiz / L**3
    k[7, 11] = -6 * eiz / L**2
    k[11, 1] = 6 * eiz / L**2
    k[11, 5] = 2 * eiz / L
    k[11, 7] = -6 * eiz / L**2
    k[11, 11] = 4 * eiz / L

    eiy = E * Iy
    k[2, 2] = 12 * eiy / L**3
    k[2, 4] = -6 * eiy / L**2
    k[2, 8] = -12 * eiy / L**3
    k[2, 10] = -6 * eiy / L**2
    k[4, 2] = -6 * eiy / L**2
    k[4, 4] = 4 * eiy / L
    k[4, 8] = 6 * eiy / L**2
    k[4, 10] = 2 * eiy / L
    k[8, 2] = -12 * eiy / L**3
    k[8, 4] = 6 * eiy / L**2
    k[8, 8] = 12 * eiy / L**3
    k[8, 10] = 6 * eiy / L**2
    k[10, 2] = -6 * eiy / L**2
    k[10, 4] = 2 * eiy / L
    k[10, 8] = 6 * eiy / L**2
    k[10, 10] = 4 * eiy / L

    return 0.5 * (k + k.T)


def global_stiffness(elem: BeamElement) -> np.ndarray:
    k_local = local_stiffness(elem)
    T = transformation_matrix(elem)
    return T.T @ k_local @ T
