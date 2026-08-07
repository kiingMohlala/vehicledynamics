"""Constant-radius fillet approximation between two planar-ish surfaces."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.geometry.vector import normalize, cross


@dataclass
class FilletSurface:
    """
    Approximate circular fillet along u, blending from surface edge A to B.
    radius: fillet radius (m)
    """
    surface_a: object
    surface_b: object
    radius: float = 0.02

    def evaluate(self, u: float, v: float) -> np.ndarray:
        u = float(np.clip(u, 0, 1))
        v = float(np.clip(v, 0, 1))
        pa = np.asarray(self.surface_a.evaluate(u, 1.0), dtype=float)
        pb = np.asarray(self.surface_b.evaluate(u, 0.0), dtype=float)
        mid = 0.5 * (pa + pb)
        # offset along average normal
        na = self.surface_a.normal(u, 1.0) if hasattr(self.surface_a, "normal") else np.array([0, 0, 1.0])
        nb = self.surface_b.normal(u, 0.0) if hasattr(self.surface_b, "normal") else np.array([0, 0, 1.0])
        n = normalize(na + nb)
        # arc from pa to pb
        angle = v * np.pi / 2  # quarter-ish
        # chord direction
        chord = pb - pa
        # approximate arc center
        center = mid + n * self.radius
        # points on arc
        e1 = normalize(pa - center)
        e2 = normalize(pb - center)
        # slerp-like
        w = (1 - v) * e1 + v * e2
        w = normalize(w)
        return center + w * self.radius

    def normal(self, u: float, v: float, eps: float = 1e-4) -> np.ndarray:
        pu = self.evaluate(min(1, u + eps), v) - self.evaluate(max(0, u - eps), v)
        pv = self.evaluate(u, min(1, v + eps)) - self.evaluate(u, max(0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 16, nv: int = 12) -> np.ndarray:
        g = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                g[i, j] = self.evaluate(u, v)
        return g
