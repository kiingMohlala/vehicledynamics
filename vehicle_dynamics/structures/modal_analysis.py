"""Simple modal analysis for free vibration of frames."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .stiffness_matrix import assemble_frame
from .materials import steel


@dataclass
class ModalResult:
    frequencies_hz: np.ndarray
    mode_shapes: np.ndarray
    n_modes: int
    success: bool


def lumped_mass_matrix(nodes: dict[str, np.ndarray], total_mass: float) -> np.ndarray:
    n = len(nodes)
    m_each = total_mass / max(n, 1)
    M = np.zeros((6 * n, 6 * n))
    for i in range(n):
        for d in range(3):  # translational mass only
            M[6 * i + d, 6 * i + d] = m_each
        for d in range(3, 6):
            M[6 * i + d, 6 * i + d] = m_each * 0.01  # small rotary inertia
    return M


def solve_modes(
    nodes: dict[str, np.ndarray],
    elements: list[tuple[str, str]],
    fixed: list[str],
    total_mass: float = 50.0,
    n_modes: int = 6,
) -> ModalResult:
    K, names = assemble_frame(nodes, elements)
    M = lumped_mass_matrix(nodes, total_mass)
    idx = {n: i for i, n in enumerate(names)}
    free = np.ones(K.shape[0], dtype=bool)
    for tag in fixed:
        if tag in idx:
            free[6 * idx[tag]:6 * idx[tag] + 6] = False
    Kff = K[np.ix_(free, free)]
    Mff = M[np.ix_(free, free)]
    try:
        # generalized eigenproblem K x = w^2 M x
        # use inv(M) K for simplicity (diagonal M)
        A = np.linalg.solve(Mff + 1e-12 * np.eye(Mff.shape[0]), Kff)
        w2, vecs = np.linalg.eig(A)
        w2 = np.real(w2)
        order = np.argsort(w2)
        w2 = w2[order]
        vecs = np.real(vecs[:, order])
        # positive frequencies
        pos = w2 > 1e-6
        w2 = w2[pos][:n_modes]
        vecs = vecs[:, pos][:, :n_modes]
        freq = np.sqrt(np.maximum(w2, 0)) / (2 * np.pi)
        return ModalResult(freq, vecs, len(freq), True)
    except Exception:
        return ModalResult(np.array([]), np.zeros((0, 0)), 0, False)
