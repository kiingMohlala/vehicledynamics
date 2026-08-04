"""Local ↔ global coordinate transformation for beam elements."""

from __future__ import annotations

import numpy as np
from .beam import BeamElement


def rotation_matrix(elem: BeamElement) -> np.ndarray:
    """
    3×3 rotation from local to global: v_global = R @ v_local.

    Local x along the beam. Local y, z chosen with a stable reference
    to avoid singularities for near-vertical members.
    """
    x_axis = elem.direction()
    # Reference vector: prefer global Z, fall back to global Y if aligned
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x_axis, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(ref, x_axis)
    n = np.linalg.norm(y_axis)
    if n < 1e-12:
        ref = np.array([1.0, 0.0, 0.0])
        y_axis = np.cross(ref, x_axis)
        n = np.linalg.norm(y_axis)
    y_axis = y_axis / n
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    # Rows are local unit vectors in global coords
    R = np.vstack([x_axis, y_axis, z_axis])
    return R  # local = R @ global, global = R.T @ local


def transformation_matrix(elem: BeamElement) -> np.ndarray:
    """
    12×12 DOF transformation: u_local = T @ u_global.

    For each node: [ux,uy,uz,rx,ry,rz]_local = blkdiag(R,R) @ [...]_global
    """
    R = rotation_matrix(elem)
    T = np.zeros((12, 12))
    for i in range(4):
        T[3 * i : 3 * i + 3, 3 * i : 3 * i + 3] = R
    return T
