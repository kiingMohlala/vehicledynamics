"""
Geometric primitives for suspension kinematics.

Instant center of a planar double wishbone is the intersection of the
extended upper and lower control-arm lines (side view or front view),
not the midpoint of either arm.
"""

from __future__ import annotations

import numpy as np
from .hardpoints import Point3


def line_intersect_2d(
    p1: np.ndarray, p2: np.ndarray,
    p3: np.ndarray, p4: np.ndarray,
    eps: float = 1e-12,
) -> np.ndarray | None:
    """
    Intersection of lines (p1–p2) and (p3–p4) in 2D.
    Returns None if parallel.
    """
    d1 = p2 - p1
    d2 = p4 - p3
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < eps:
        return None
    t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / cross
    return p1 + t * d1


def instant_center_yz(
    upper_inner_y: float, upper_inner_z: float,
    upper_outer_y: float, upper_outer_z: float,
    lower_inner_y: float, lower_inner_z: float,
    lower_outer_y: float, lower_outer_z: float,
) -> tuple[float, float] | None:
    """
    Instant center in the YZ plane (front view).
    Upper and lower arms projected to YZ; IC = line intersection.
    """
    u1 = np.array([upper_inner_y, upper_inner_z])
    u2 = np.array([upper_outer_y, upper_outer_z])
    l1 = np.array([lower_inner_y, lower_inner_z])
    l2 = np.array([lower_outer_y, lower_outer_z])
    ic = line_intersect_2d(u1, u2, l1, l2)
    if ic is None:
        return None
    return float(ic[0]), float(ic[1])


def instant_center_xz(
    upper_front: Point3, upper_rear: Point3,
    lower_front: Point3, lower_rear: Point3,
) -> tuple[float, float] | None:
    """
    Instant center in the XZ plane (side view) using arm inner pivots
    as representatives of the body attachments.
    """
    u1 = np.array([upper_front.x, upper_front.z])
    u2 = np.array([upper_rear.x, upper_rear.z])
    l1 = np.array([lower_front.x, lower_front.z])
    l2 = np.array([lower_rear.x, lower_rear.z])
    ic = line_intersect_2d(u1, u2, l1, l2)
    if ic is None:
        return None
    return float(ic[0]), float(ic[1])


def average_inner(front: Point3, rear: Point3) -> Point3:
    return Point3(
        0.5 * (front.x + rear.x),
        0.5 * (front.y + rear.y),
        0.5 * (front.z + rear.z),
    )


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v * 0.0
    return v / n
