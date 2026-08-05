"""Reference path generation and queries."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class PathPoint:
    x: float
    y: float
    psi: float = 0.0
    s: float = 0.0
    kappa: float = 0.0
    v_ref: float = 15.0


@dataclass
class ReferencePath:
    points: list[PathPoint]

    def __post_init__(self) -> None:
        if not self.points:
            self.points = [PathPoint(0.0, 0.0)]
        self._build()

    def _build(self) -> None:
        pts = self.points
        s = 0.0
        pts[0].s = 0.0
        for i in range(1, len(pts)):
            dx = pts[i].x - pts[i - 1].x
            dy = pts[i].y - pts[i - 1].y
            s += float(np.hypot(dx, dy))
            pts[i].s = s
            if abs(dx) + abs(dy) > 1e-9:
                pts[i - 1].psi = float(np.arctan2(dy, dx))
        pts[-1].psi = pts[-2].psi if len(pts) > 1 else 0.0
        self._s = np.array([p.s for p in pts])
        self._x = np.array([p.x for p in pts])
        self._y = np.array([p.y for p in pts])
        self._psi = np.array([p.psi for p in pts])
        self._v = np.array([p.v_ref for p in pts])
        self.length = float(self._s[-1]) if len(self._s) else 0.0

    def sample(self, s: float) -> PathPoint:
        s = float(np.clip(s, 0.0, max(self.length, 0.0)))
        x = float(np.interp(s, self._s, self._x))
        y = float(np.interp(s, self._s, self._y))
        psi = float(np.interp(s, self._s, self._psi))
        v = float(np.interp(s, self._s, self._v))
        return PathPoint(x=x, y=y, psi=psi, s=s, v_ref=v)

    def nearest(self, x: float, y: float) -> tuple[PathPoint, float]:
        """Return nearest path point and signed cross-track error."""
        dx = self._x - x
        dy = self._y - y
        i = int(np.argmin(dx * dx + dy * dy))
        p = self.points[i]
        # Cross-track: left positive relative to path heading
        ex = x - p.x
        ey = y - p.y
        cte = -np.sin(p.psi) * ex + np.cos(p.psi) * ey
        return p, float(cte)


def make_straight(length: float = 200.0, v_ref: float = 20.0, n: int = 50) -> ReferencePath:
    xs = np.linspace(0.0, length, n)
    pts = [PathPoint(x=float(x), y=0.0, psi=0.0, v_ref=v_ref) for x in xs]
    return ReferencePath(pts)


def make_circle(radius: float = 40.0, v_ref: float = 15.0, n: int = 120) -> ReferencePath:
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    pts = [
        PathPoint(
            x=float(radius * np.cos(t)),
            y=float(radius * np.sin(t)),
            psi=float(t + np.pi / 2),
            v_ref=v_ref,
        )
        for t in th
    ]
    # Close loop
    pts.append(PathPoint(x=pts[0].x, y=pts[0].y, psi=pts[0].psi, v_ref=v_ref))
    return ReferencePath(pts)


def make_slalom(
    length: float = 120.0,
    amplitude: float = 3.0,
    wavelength: float = 30.0,
    v_ref: float = 15.0,
    n: int = 100,
) -> ReferencePath:
    xs = np.linspace(0.0, length, n)
    ys = amplitude * np.sin(2 * np.pi * xs / wavelength)
    pts = [PathPoint(x=float(x), y=float(y), v_ref=v_ref) for x, y in zip(xs, ys)]
    return ReferencePath(pts)


def make_figure_eight(a: float = 30.0, v_ref: float = 12.0, n: int = 160) -> ReferencePath:
    t = np.linspace(0.0, 2 * np.pi, n)
    # Lemniscate of Gerono
    xs = a * np.sin(t)
    ys = a * np.sin(t) * np.cos(t)
    pts = [PathPoint(x=float(x), y=float(y), v_ref=v_ref) for x, y in zip(xs, ys)]
    return ReferencePath(pts)


def make_waypoints(
    waypoints: list[tuple[float, float]],
    v_ref: float = 15.0,
) -> ReferencePath:
    pts = [PathPoint(x=float(x), y=float(y), v_ref=v_ref) for x, y in waypoints]
    return ReferencePath(pts)
