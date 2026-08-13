"""Simple packaging / clearance checks between points and spheres."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ClearanceResult:
    pair: str
    distance: float
    min_required: float
    ok: bool


def point_clearance(a: np.ndarray, b: np.ndarray, min_dist: float = 0.02) -> ClearanceResult:
    d = float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))
    return ClearanceResult(pair="points", distance=d, min_required=min_dist, ok=d >= min_dist)


def check_corner_packaging(hp_points: dict, min_dist: float = 0.03) -> list[ClearanceResult]:
    """Check key pairs: strut vs tierod, UCA vs LCA outer, wheel vs chassis pickups."""
    results = []
    pairs = [
        ("strut_lower", "tierod_outer"),
        ("UCA_outer", "LCA_outer"),
        ("wheel_center", "LCA_front"),
        ("wheel_center", "UCA_front"),
    ]
    for a, b in pairs:
        if a in hp_points and b in hp_points:
            r = point_clearance(hp_points[a], hp_points[b], min_dist=min_dist)
            r.pair = f"{a}-{b}"
            results.append(r)
    return results
