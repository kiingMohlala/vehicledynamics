"""Track geometry: centerline, boundaries, banking, elevation, grip."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .track_segments import TrackSegment, SurfaceProperties
from .curvature import curvature_from_xy, reference_speed
from .friction_map import FrictionMap
from .elevation import elevation_at


@dataclass
class Track:
    name: str
    segments: list[TrackSegment]
    # Discretized centerline
    s: np.ndarray = field(default_factory=lambda: np.zeros(1))
    x: np.ndarray = field(default_factory=lambda: np.zeros(1))
    y: np.ndarray = field(default_factory=lambda: np.zeros(1))
    z: np.ndarray = field(default_factory=lambda: np.zeros(1))
    heading: np.ndarray = field(default_factory=lambda: np.zeros(1))
    width: np.ndarray = field(default_factory=lambda: np.ones(1) * 12.0)
    banking_deg: np.ndarray = field(default_factory=lambda: np.zeros(1))
    curvature: np.ndarray = field(default_factory=lambda: np.zeros(1))
    friction: FrictionMap | None = None
    closed: bool = True
    ds: float = 1.0

    @property
    def length(self) -> float:
        return float(self.s[-1]) if len(self.s) else 0.0

    def build(self, ds: float = 1.0) -> "Track":
        """Discretize segments into a polyline centerline."""
        self.ds = ds
        xs, ys, zs, hs, ws, banks, ss = [], [], [], [], [], [], []
        x = y = z = h = s_acc = 0.0
        mus_len, mus_val = [], []

        for seg in self.segments:
            n = max(2, int(np.ceil(seg.length_m / ds)))
            step = seg.length_m / n
            kappa = seg.curvature
            dpsi = kappa * step
            for _ in range(n):
                xs.append(x); ys.append(y); zs.append(z); hs.append(h)
                ws.append(seg.width_m); banks.append(seg.banking_deg); ss.append(s_acc)
                x += step * np.cos(h)
                y += step * np.sin(h)
                h += dpsi
                z += seg.elevation_change_m / n
                s_acc += step
            mus_len.append(seg.length_m)
            mus_val.append(seg.surface.mu)

        self.x = np.array(xs); self.y = np.array(ys); self.z = np.array(zs)
        self.heading = np.array(hs); self.width = np.array(ws)
        self.banking_deg = np.array(banks); self.s = np.array(ss)
        self.curvature = curvature_from_xy(self.x, self.y)
        self.friction = FrictionMap.from_segments(mus_len, mus_val)
        if self.closed and len(self.x) > 2:
            # soft close: do not force geometry, just mark
            pass
        return self

    def left_boundary(self) -> tuple[np.ndarray, np.ndarray]:
        n = self.heading + np.pi / 2
        return self.x + 0.5 * self.width * np.cos(n), self.y + 0.5 * self.width * np.sin(n)

    def right_boundary(self) -> tuple[np.ndarray, np.ndarray]:
        n = self.heading - np.pi / 2
        return self.x + 0.5 * self.width * np.cos(n), self.y + 0.5 * self.width * np.sin(n)

    def mu_at(self, s: float) -> float:
        if self.friction is None:
            return 1.0
        return float(self.friction.mu(s)[0])

    def sample_at(self, s_query: float) -> dict:
        s_query = float(np.clip(s_query, 0.0, self.length))
        i = int(np.searchsorted(self.s, s_query, side="right") - 1)
        i = int(np.clip(i, 0, len(self.s) - 1))
        return {
            "s": s_query,
            "x": float(self.x[i]),
            "y": float(self.y[i]),
            "z": float(self.z[i]),
            "heading": float(self.heading[i]),
            "width": float(self.width[i]),
            "banking_deg": float(self.banking_deg[i]),
            "curvature": float(self.curvature[i]),
            "mu": self.mu_at(s_query),
        }
