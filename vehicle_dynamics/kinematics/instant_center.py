"""Instant center from control-arm line intersections (2D side / front view)."""
from __future__ import annotations

import numpy as np


def line_intersection_2d(p1, d1, p2, d2) -> np.ndarray | None:
    """Intersection of lines p1+t*d1 and p2+s*d2 in 2D."""
    A = np.column_stack([d1, -d2])
    if abs(np.linalg.det(A)) < 1e-12:
        return None
    t = np.linalg.solve(A, p2 - p1)[0]
    return p1 + t * d1


def front_view_ic(lca_inner_yz, lca_outer_yz, uca_inner_yz, uca_outer_yz) -> np.ndarray:
    """
    Instant center in YZ (front view) from upper/lower arm projections.
    Returns (y, z). If parallel, returns far-away point.
    """
    d_l = lca_outer_yz - lca_inner_yz
    d_u = uca_outer_yz - uca_inner_yz
    ic = line_intersection_2d(lca_inner_yz, d_l, uca_inner_yz, d_u)
    if ic is None:
        # parallel → IC at infinity (approximate)
        mid = 0.5 * (lca_outer_yz + uca_outer_yz)
        return mid + np.array([1000.0, 0.0])
    return ic


def side_view_ic(lca_front_xz, lca_rear_xz, uca_front_xz, uca_rear_xz) -> np.ndarray:
    """Instant center in XZ (side view)."""
    d_l = lca_rear_xz - lca_front_xz
    d_u = uca_rear_xz - uca_front_xz
    ic = line_intersection_2d(lca_front_xz, d_l, uca_front_xz, d_u)
    if ic is None:
        mid = 0.5 * (lca_front_xz + uca_front_xz)
        return mid + np.array([1000.0, 0.0])
    return ic
