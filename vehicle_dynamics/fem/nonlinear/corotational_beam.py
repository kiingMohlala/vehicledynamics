"""
Geometrically nonlinear beam helpers (total Lagrangian + chord stretch).

Material response: linear K on the undeformed configuration.
Geometric nonlinearity: axial force from chord stretch drives K_G.
"""

from __future__ import annotations

import numpy as np
from vehicle_dynamics.fem.beam import BeamElement
from vehicle_dynamics.fem.stiffness import global_stiffness


def undeformed_length(elem: BeamElement) -> float:
    return elem.length()


def current_geometry(elem: BeamElement, u: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    ui = u[elem.node_i.dof_indices()]
    uj = u[elem.node_j.dof_indices()]
    pi = elem.node_i.coords() + ui[:3]
    pj = elem.node_j.coords() + uj[:3]
    L = float(np.linalg.norm(pj - pi))
    if L < 1e-14:
        raise ValueError(f"Element {elem.id} collapsed (L≈0)")
    return L, pi, pj


def rotation_from_chord(pi: np.ndarray, pj: np.ndarray) -> np.ndarray:
    ex = (pj - pi) / np.linalg.norm(pj - pi)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(ex, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    ey = np.cross(ref, ex)
    ey = ey / (np.linalg.norm(ey) + 1e-30)
    ez = np.cross(ex, ey)
    ez = ez / (np.linalg.norm(ez) + 1e-30)
    return np.vstack([ex, ey, ez])


def axial_force(elem: BeamElement, u: np.ndarray) -> float:
    """N = EA/L0 * (L - L0), tension > 0."""
    L0 = undeformed_length(elem)
    L, _, _ = current_geometry(elem, u)
    EA = elem.material.E * elem.section.A
    return EA / L0 * (L - L0)


def internal_force_global(elem: BeamElement, u: np.ndarray) -> np.ndarray:
    """F_int = k_global_linear @ u_elem (total Lagrangian linear material)."""
    ke = global_stiffness(elem)
    dofs_u = np.concatenate(
        [u[elem.node_i.dof_indices()], u[elem.node_j.dof_indices()]]
    )
    return ke @ dofs_u


def material_stiffness_global(elem: BeamElement, u: np.ndarray) -> np.ndarray:
    return global_stiffness(elem)
