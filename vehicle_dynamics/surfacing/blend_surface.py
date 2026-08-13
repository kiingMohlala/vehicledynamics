"""G1/G2-style blending between two surfaces along a shared edge (v direction)."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.geometry.vector import normalize, cross


@dataclass
class BlendSurface:
    """
    Hermite blend between surface A (at v=0 side) and surface B (at v=1 side).
    At v=0 matches A(u,1) with tangent from A; at v=1 matches B(u,0).
    """
    surface_a: object
    surface_b: object
    blend_width: float = 1.0

    def evaluate(self, u: float, v: float) -> np.ndarray:
        u = float(np.clip(u, 0, 1))
        v = float(np.clip(v, 0, 1))
        # cubic Hermite
        p0 = np.asarray(self.surface_a.evaluate(u, 1.0), dtype=float)
        p1 = np.asarray(self.surface_b.evaluate(u, 0.0), dtype=float)
        # finite-difference tangents in v
        eps = 1e-3
        t0 = (np.asarray(self.surface_a.evaluate(u, 1.0), dtype=float) -
              np.asarray(self.surface_a.evaluate(u, max(0, 1 - eps)), dtype=float)) / eps
        t1 = (np.asarray(self.surface_b.evaluate(u, min(1, eps)), dtype=float) -
              np.asarray(self.surface_b.evaluate(u, 0.0), dtype=float)) / eps
        h00 = 2 * v**3 - 3 * v**2 + 1
        h10 = v**3 - 2 * v**2 + v
        h01 = -2 * v**3 + 3 * v**2
        h11 = v**3 - v**2
        return h00 * p0 + h10 * t0 * self.blend_width + h01 * p1 + h11 * t1 * self.blend_width

    def normal(self, u: float, v: float, eps: float = 1e-4) -> np.ndarray:
        pu = self.evaluate(min(1, u + eps), v) - self.evaluate(max(0, u - eps), v)
        pv = self.evaluate(u, min(1, v + eps)) - self.evaluate(u, max(0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 20, nv: int = 20) -> np.ndarray:
        g = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                g[i, j] = self.evaluate(u, v)
        return g
