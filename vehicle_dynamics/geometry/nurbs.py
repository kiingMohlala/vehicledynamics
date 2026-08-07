"""NURBS curves."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .splines import open_uniform_knots, cox_de_boor
from .vector import normalize


@dataclass
class NurbsCurve:
    control_points: np.ndarray
    weights: np.ndarray | None = None
    degree: int = 3
    knots: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.control_points = np.asarray(self.control_points, dtype=float)
        n = self.control_points.shape[0]
        self.degree = min(int(self.degree), max(1, n - 1))
        if self.weights is None:
            self.weights = np.ones(n)
        else:
            self.weights = np.asarray(self.weights, dtype=float)
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
        num = np.zeros(3)
        den = 0.0
        for i in range(n):
            b = cox_de_boor(i, p, u, self.knots) * self.weights[i]
            num += b * self.control_points[i]
            den += b
        if den < 1e-15:
            return self.control_points[0].copy()
        return num / den

    def tangent(self, u: float, eps: float = 1e-5) -> np.ndarray:
        u0, u1 = max(0.0, u - eps), min(1.0, u + eps)
        return normalize(self.evaluate(u1) - self.evaluate(u0))

    def length(self, n: int = 100) -> float:
        pts = self.sample(n)
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))

    def sample(self, n: int = 50) -> np.ndarray:
        return np.array([self.evaluate(u) for u in np.linspace(0, 1, n)])
