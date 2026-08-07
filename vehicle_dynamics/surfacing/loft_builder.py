"""Multi-section loft surfaces."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence
import numpy as np

from vehicle_dynamics.geometry.curves import BezierCurve, Line
from vehicle_dynamics.geometry.surfaces import LoftSurface, BilinearSurface
from vehicle_dynamics.geometry.vector import normalize, cross


@dataclass
class MultiLoftSurface:
    """Loft through N profile curves by linear blending of adjacent lofts."""
    profiles: list  # curve-like with evaluate(t)
    # cached pairwise lofts

    def __post_init__(self) -> None:
        if len(self.profiles) < 2:
            raise ValueError("Need at least 2 profiles")
        self._lofts = [
            LoftSurface(self.profiles[i], self.profiles[i + 1])
            for i in range(len(self.profiles) - 1)
        ]

    def evaluate(self, u: float, v: float) -> np.ndarray:
        u = float(np.clip(u, 0.0, 1.0))
        v = float(np.clip(v, 0.0, 1.0))
        nseg = len(self._lofts)
        # v selects which segment
        fv = v * nseg
        i = min(int(fv), nseg - 1)
        local_v = fv - i
        return self._lofts[i].evaluate(u, local_v)

    def normal(self, u: float, v: float, eps: float = 1e-4) -> np.ndarray:
        p = self.evaluate(u, v)
        pu = self.evaluate(min(1, u + eps), v) - self.evaluate(max(0, u - eps), v)
        pv = self.evaluate(u, min(1, v + eps)) - self.evaluate(u, max(0, v - eps))
        return normalize(cross(pu, pv))

    def sample_grid(self, nu: int = 20, nv: int = 20) -> np.ndarray:
        grid = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                grid[i, j] = self.evaluate(u, v)
        return grid


def loft_from_points(sections: Sequence[np.ndarray]) -> MultiLoftSurface:
    """
    sections: list of (n_pts, 3) polylines — converted to Bézier via uniform sampling.
    """
    curves = []
    for sec in sections:
        sec = np.asarray(sec, dtype=float)
        if sec.shape[0] == 2:
            curves.append(Line(sec[0], sec[1]))
        else:
            # take 4 control points by sampling
            idx = np.linspace(0, len(sec) - 1, 4).astype(int)
            curves.append(BezierCurve(sec[idx]))
    return MultiLoftSurface(curves)
