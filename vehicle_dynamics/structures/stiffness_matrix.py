"""Assemble simple multi-beam global stiffness (educational / metrics)."""
from __future__ import annotations

import numpy as np
from .beam_elements import local_beam_stiffness
from .materials import StructuralMaterial, steel


def transform_12(R3: np.ndarray) -> np.ndarray:
    T = np.zeros((12, 12))
    for i in range(4):
        T[i * 3:(i + 1) * 3, i * 3:(i + 1) * 3] = R3
    return T


def beam_rotation(p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    x = p1 - p0
    L = np.linalg.norm(x)
    x = x / max(L, 1e-15)
    # build orthonormal triad
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x, up)) > 0.9:
        up = np.array([0.0, 1.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y) + 1e-15
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def assemble_frame(
    nodes: dict[str, np.ndarray],
    elements: list[tuple[str, str]],
    mat: StructuralMaterial | None = None,
    A: float = 5e-4,
    Iy: float = 1e-7,
    Iz: float = 1e-7,
    J: float = 2e-7,
) -> tuple[np.ndarray, list[str]]:
    """Return K_global and node name order."""
    mat = mat or steel()
    names = list(nodes.keys())
    idx = {n: i for i, n in enumerate(names)}
    ndof = 6 * len(names)
    K = np.zeros((ndof, ndof))
    for a, b in elements:
        p0, p1 = nodes[a], nodes[b]
        L = float(np.linalg.norm(p1 - p0))
        if L < 1e-12:
            continue
        kloc = local_beam_stiffness(L, mat, A, Iy, Iz, J)
        R = beam_rotation(p0, p1)
        T = transform_12(R)
        kg = T.T @ kloc @ T
        dofs = []
        for n in (a, b):
            base = 6 * idx[n]
            dofs.extend(range(base, base + 6))
        for i in range(12):
            for j in range(12):
                K[dofs[i], dofs[j]] += kg[i, j]
    return K, names
