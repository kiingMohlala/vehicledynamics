"""B-spline curves."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .vector import normalize


def open_uniform_knots(n_ctrl: int, degree: int) -> np.ndarray:
    """Clamped open-uniform knot vector of length n_ctrl + degree + 1."""
    p = int(degree)
    n = int(n_ctrl)
    m = n + p + 1
    knots = np.zeros(m, dtype=float)
    # clamped ends
    knots[n:] = 1.0
    # interior
    n_interior = n - p - 1
    if n_interior > 0:
        knots[p + 1 : n] = np.arange(1, n_interior + 1) / (n_interior + 1)
    return knots


def cox_de_boor(i: int, p: int, u: float, knots: np.ndarray) -> float:
    if p == 0:
        return 1.0 if knots[i] <= u < knots[i + 1] or (u == knots[-1] and knots[i] <= u <= knots[i + 1]) else 0.0
    d1 = knots[i + p] - knots[i]
    d2 = knots[i + p + 1] - knots[i + 1]
    c1 = 0.0 if d1 < 1e-15 else (u - knots[i]) / d1 * cox_de_boor(i, p - 1, u, knots)
    c2 = 0.0 if d2 < 1e-15 else (knots[i + p + 1] - u) / d2 * cox_de_boor(i + 1, p - 1, u, knots)
    return c1 + c2


@dataclass
class BSplineCurve:
    control_points: np.ndarray
    degree: int = 3
    knots: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.control_points = np.asarray(self.control_points, dtype=float)
        n = self.control_points.shape[0]
        # auto-reduce degree if too few controls
        self.degree = min(self.degree, max(1, n - 1))
        if self.knots is None:
            self.knots = open_uniform_knots(n, self.degree)
        else:
            self.knots = np.asarray(self.knots, dtype=float)

    def evaluate(self, u: float) -> np.ndarray:
        u = float(np.clip(u, 0.0, 1.0))
        n = self.control_points.shape[0]
        p = self.degree
        if u >= 1.0 - 1e-15:
            return self.control_points[-1].copy()
        if u <= 1e-15:
            return self.control_points[0].copy()
        pt = np.zeros(3)
        for i in range(n):
            b = cox_de_boor(i, p, u, self.knots)
            pt += b * self.control_points[i]
        return pt

    def tangent(self, u: float, eps: float = 1e-5) -> np.ndarray:
        u0, u1 = max(0.0, u - eps), min(1.0, u + eps)
        return normalize(self.evaluate(u1) - self.evaluate(u0))

    def length(self, n: int = 100) -> float:
        pts = self.sample(n)
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))

    def sample(self, n: int = 50) -> np.ndarray:
        return np.array([self.evaluate(u) for u in np.linspace(0, 1, n)])
