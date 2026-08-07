"""Standard parametric body panel templates."""
from __future__ import annotations

import numpy as np
from vehicle_dynamics.geometry.curves import BezierCurve, Line
from vehicle_dynamics.geometry.surfaces import LoftSurface, BilinearSurface

from .panel import Panel
from .loft_builder import MultiLoftSurface


def _bezier_xy(pts, z_func=None):
    pts = np.asarray(pts, dtype=float)
    if pts.shape[1] == 2:
        z = np.zeros((len(pts), 1)) if z_func is None else np.array([[z_func(p[0], p[1])] for p in pts])
        pts = np.hstack([pts, z])
    return BezierCurve(pts)


def hood_panel(wheelbase: float = 2.7, width: float = 1.6, height: float = 0.85) -> Panel:
    # two profiles front→rear of hood
    y = width * 0.5
    front = BezierCurve([[0.15, -y * 0.9, height * 0.55], [0.15, 0, height * 0.58], [0.15, y * 0.9, height * 0.55]])
    # cubic needs 4 pts - use 4 control points across y
    front = BezierCurve([
        [0.20, -y, height * 0.55],
        [0.20, -y * 0.3, height * 0.60],
        [0.20, y * 0.3, height * 0.60],
        [0.20, y, height * 0.55],
    ])
    rear = BezierCurve([
        [0.55 * wheelbase, -y * 0.95, height * 0.70],
        [0.55 * wheelbase, -y * 0.3, height * 0.75],
        [0.55 * wheelbase, y * 0.3, height * 0.75],
        [0.55 * wheelbase, y * 0.95, height * 0.70],
    ])
    return Panel("hood", "hood", LoftSurface(front, rear))


def roof_panel(wheelbase: float = 2.7, width: float = 1.4, height: float = 1.15) -> Panel:
    y = width * 0.5
    front = BezierCurve([
        [0.55 * wheelbase, -y, height * 0.95],
        [0.55 * wheelbase, -y * 0.3, height],
        [0.55 * wheelbase, y * 0.3, height],
        [0.55 * wheelbase, y, height * 0.95],
    ])
    rear = BezierCurve([
        [0.85 * wheelbase, -y * 0.9, height * 0.90],
        [0.85 * wheelbase, -y * 0.3, height * 0.95],
        [0.85 * wheelbase, y * 0.3, height * 0.95],
        [0.85 * wheelbase, y * 0.9, height * 0.90],
    ])
    return Panel("roof", "roof", LoftSurface(front, rear))


def door_panel(side: float = 1.0, wheelbase: float = 2.7, height: float = 0.90) -> Panel:
    """side = +1 left, -1 right."""
    y = 0.95 * side
    bottom = BezierCurve([
        [0.45 * wheelbase, y, 0.20],
        [0.55 * wheelbase, y, 0.18],
        [0.70 * wheelbase, y, 0.18],
        [0.80 * wheelbase, y, 0.22],
    ])
    top = BezierCurve([
        [0.50 * wheelbase, y * 0.95, height * 0.85],
        [0.58 * wheelbase, y * 0.95, height * 0.90],
        [0.72 * wheelbase, y * 0.95, height * 0.88],
        [0.82 * wheelbase, y * 0.95, height * 0.80],
    ])
    return Panel(f"door_{'L' if side > 0 else 'R'}", "door", LoftSurface(bottom, top))


def fender_panel(front: bool = True, side: float = 1.0, wheelbase: float = 2.7) -> Panel:
    y = 0.95 * side
    x0 = 0.05 if front else 0.85 * wheelbase
    x1 = 0.40 * wheelbase if front else 1.05 * wheelbase
    inner = BezierCurve([[x0, y * 0.7, 0.35], [0.5 * (x0 + x1), y * 0.7, 0.40], [0.5 * (x0 + x1), y * 0.7, 0.45], [x1, y * 0.7, 0.35]])
    outer = BezierCurve([[x0, y, 0.30], [0.5 * (x0 + x1), y, 0.50], [0.5 * (x0 + x1), y, 0.55], [x1, y, 0.30]])
    name = f"{'front' if front else 'rear'}_fender_{'L' if side > 0 else 'R'}"
    return Panel(name, "fender", LoftSurface(inner, outer))


def floor_panel(wheelbase: float = 2.7, width: float = 1.5) -> Panel:
    y = width * 0.5
    a = BilinearSurface(
        np.array([0.0, -y, 0.08]), np.array([wheelbase, -y, 0.08]),
        np.array([0.0, y, 0.08]), np.array([wheelbase, y, 0.08]),
    )
    return Panel("floor", "floor", a)


def undertray_panel(wheelbase: float = 2.7, width: float = 1.5) -> Panel:
    y = width * 0.5
    a = BilinearSurface(
        np.array([0.1, -y, 0.05]), np.array([wheelbase * 0.9, -y, 0.04]),
        np.array([0.1, y, 0.05]), np.array([wheelbase * 0.9, y, 0.04]),
    )
    return Panel("undertray", "undertray", a)


def splitter_panel(width: float = 1.8) -> Panel:
    y = width * 0.5
    a = BilinearSurface(
        np.array([-0.15, -y, 0.06]), np.array([0.15, -y, 0.06]),
        np.array([-0.15, y, 0.06]), np.array([0.15, y, 0.06]),
    )
    return Panel("splitter", "splitter", a)


def diffuser_panel(wheelbase: float = 2.7, width: float = 1.4) -> Panel:
    y = width * 0.5
    front = BezierCurve([
        [0.85 * wheelbase, -y, 0.05],
        [0.85 * wheelbase, -y * 0.3, 0.05],
        [0.85 * wheelbase, y * 0.3, 0.05],
        [0.85 * wheelbase, y, 0.05],
    ])
    rear = BezierCurve([
        [1.05 * wheelbase, -y, 0.18],
        [1.05 * wheelbase, -y * 0.3, 0.20],
        [1.05 * wheelbase, y * 0.3, 0.20],
        [1.05 * wheelbase, y, 0.18],
    ])
    return Panel("diffuser", "diffuser", LoftSurface(front, rear))


def wing_panel(x: float = 2.6, span: float = 1.5, chord: float = 0.30, z: float = 1.0) -> Panel:
    y = span * 0.5
    le = BezierCurve([[x, -y, z], [x, -y * 0.3, z + 0.02], [x, y * 0.3, z + 0.02], [x, y, z]])
    te = BezierCurve([[x + chord, -y, z - 0.02], [x + chord, -y * 0.3, z], [x + chord, y * 0.3, z], [x + chord, y, z - 0.02]])
    return Panel("rear_wing", "wing", LoftSurface(le, te))
