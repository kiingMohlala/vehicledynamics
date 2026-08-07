"""Sweep a profile along a guide curve."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.geometry.vector import normalize, cross


@dataclass
class SweepSurface:
    path: object  # curve with evaluate(t), tangent(t)
    profile: object  # curve in local 2D/3D evaluated in path frame
    profile_scale: float = 1.0

    def _frame(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        origin = np.asarray(self.path.evaluate(t), dtype=float)
        T = normalize(self.path.tangent(t))
        # up hint
        up = np.array([0.0, 0.0, 1.0])
        B = cross(T, up)
        if np.linalg.norm(B) < 1e-10:
            B = cross(T, np.array([0.0, 1.0, 0.0]))
        B = normalize(B)
        N = cross(B, T)
        return origin, T, N, B

    def evaluate(self, u: float, v: float) -> np.ndarray:
        origin, T, N, B = self._frame(v)
        # profile at u in local N-B plane (ignore profile z or use as offset along N/B)
        local = np.asarray(self.profile.evaluate(u), dtype=float)
        # map x->along tangent offset optional, y->B, z->N
        if local.size == 2:
            y, z = local[0], local[1]
        else:
            y, z = local[1], local[2]
        return origin + self.profile_scale * (y * B + z * N)

    def normal(self, u: float, v: float, eps: float = 1e-4) -> np.ndarray:
        pu = self.evaluate(min(1, u + eps), v) - self.evaluate(max(0, u - eps), v)
        pv = self.evaluate(u, min(1, v + eps)) - self.evaluate(u, max(0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 20, nv: int = 30) -> np.ndarray:
        grid = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                grid[i, j] = self.evaluate(u, v)
        return grid
