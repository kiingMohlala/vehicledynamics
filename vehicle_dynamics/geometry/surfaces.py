"""Surfaces: loft, simple bilinear Coons-like patch."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol
import numpy as np

from .vector import normalize, cross


class CurveLike(Protocol):
    def evaluate(self, t: float) -> np.ndarray: ...
    def sample(self, n: int = 50) -> np.ndarray: ...


@dataclass
class LoftSurface:
    """Ruled / lofted surface between two curves (linear in v)."""
    curve_a: CurveLike
    curve_b: CurveLike

    def evaluate(self, u: float, v: float) -> np.ndarray:
        a = self.curve_a.evaluate(u)
        b = self.curve_b.evaluate(u)
        return (1.0 - v) * a + v * b

    def normal(self, u: float, v: float, eps: float = 1e-5) -> np.ndarray:
        p = self.evaluate(u, v)
        pu = self.evaluate(min(1.0, u + eps), v) - self.evaluate(max(0.0, u - eps), v)
        pv = self.evaluate(u, min(1.0, v + eps)) - self.evaluate(u, max(0.0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 20, nv: int = 10) -> np.ndarray:
        """Return (nu, nv, 3) point grid."""
        grid = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                grid[i, j] = self.evaluate(u, v)
        return grid


@dataclass
class BilinearSurface:
    """Coons-style bilinear patch from 4 corners: p00, p10, p01, p11."""
    p00: np.ndarray
    p10: np.ndarray
    p01: np.ndarray
    p11: np.ndarray

    def evaluate(self, u: float, v: float) -> np.ndarray:
        return (
            (1 - u) * (1 - v) * self.p00
            + u * (1 - v) * self.p10
            + (1 - u) * v * self.p01
            + u * v * self.p11
        )

    def normal(self, u: float, v: float, eps: float = 1e-5) -> np.ndarray:
        pu = self.evaluate(min(1, u + eps), v) - self.evaluate(max(0, u - eps), v)
        pv = self.evaluate(u, min(1, v + eps)) - self.evaluate(u, max(0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 20, nv: int = 20) -> np.ndarray:
        grid = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                grid[i, j] = self.evaluate(u, v)
        return grid
