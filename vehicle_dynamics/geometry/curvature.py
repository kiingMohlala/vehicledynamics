"""Curvature analysis for curves and discrete surfaces."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .vector import norm, normalize, cross, dot


def curve_curvature(curve, t: float, eps: float = 1e-5) -> float:
    """κ = |r' × r''| / |r'|^3."""
    t0, t1, t2 = max(0, t - eps), t, min(1, t + eps)
    p0, p1, p2 = curve.evaluate(t0), curve.evaluate(t1), curve.evaluate(t2)
    rp = (p2 - p0) / (t2 - t0 + 1e-15)
    rpp = (p2 - 2 * p1 + p0) / (eps ** 2)
    return float(norm(cross(rp, rpp)) / (norm(rp) ** 3 + 1e-15))


def discrete_principal_curvatures(grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Approximate principal curvatures on a regular (nu,nv,3) grid via finite differences.
    Returns k1, k2 arrays of shape (nu-2, nv-2).
    """
    # first fundamental form approximations at interior points
    Pu = grid[2:, 1:-1] - grid[:-2, 1:-1]
    Pv = grid[1:-1, 2:] - grid[1:-1, :-2]
    Puu = grid[2:, 1:-1] - 2 * grid[1:-1, 1:-1] + grid[:-2, 1:-1]
    Pvv = grid[1:-1, 2:] - 2 * grid[1:-1, 1:-1] + grid[1:-1, :-2]
    Puv = (grid[2:, 2:] - grid[2:, :-2] - grid[:-2, 2:] + grid[:-2, :-2]) / 4.0

    nu_i, nv_i = Pu.shape[0], Pu.shape[1]
    k1 = np.zeros((nu_i, nv_i))
    k2 = np.zeros((nu_i, nv_i))
    for i in range(nu_i):
        for j in range(nv_i):
            pu, pv = Pu[i, j], Pv[i, j]
            E = float(np.dot(pu, pu))
            F = float(np.dot(pu, pv))
            G = float(np.dot(pv, pv))
            n = normalize(cross(pu, pv))
            L = float(np.dot(Puu[i, j], n))
            M = float(np.dot(Puv[i, j], n))
            N = float(np.dot(Pvv[i, j], n))
            detI = E * G - F * F
            if abs(detI) < 1e-15:
                continue
            # shape operator eigenvalues
            S = np.array([[L, M], [M, N]]) @ np.linalg.inv(np.array([[E, F], [F, G]]))
            eig = np.linalg.eigvals(S)
            k1[i, j], k2[i, j] = float(np.real(eig[0])), float(np.real(eig[1]))
    return k1, k2


def gaussian_curvature(grid: np.ndarray) -> np.ndarray:
    k1, k2 = discrete_principal_curvatures(grid)
    return k1 * k2


def mean_curvature(grid: np.ndarray) -> np.ndarray:
    k1, k2 = discrete_principal_curvatures(grid)
    return 0.5 * (k1 + k2)
