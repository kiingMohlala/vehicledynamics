"""AABB interference detection between components."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import numpy as np
from .component import Component


@dataclass
class InterferenceHit:
    a: str
    b: str
    overlap: np.ndarray  # positive = penetration per axis
    volume_proxy: float

    @property
    def penetrating(self) -> bool:
        return bool(np.all(self.overlap > 0))


def aabb_overlap(a: Component, b: Component) -> np.ndarray:
    """Return per-axis overlap depth (positive if overlapping)."""
    amin, amax = a.aabb_min, a.aabb_max
    bmin, bmax = b.aabb_min, b.aabb_max
    return np.minimum(amax, bmax) - np.maximum(amin, bmin)


def detect_interferences(
    components: Iterable[Component],
    skip_same_category: bool = False,
    min_overlap: float = 1e-4,
) -> list[InterferenceHit]:
    comps = list(components)
    hits = []
    for i in range(len(comps)):
        for j in range(i + 1, len(comps)):
            a, b = comps[i], comps[j]
            if skip_same_category and a.category == b.category:
                continue
            # allow nested packaging (e.g. seat inside cockpit) by category pairs
            if {a.category, b.category} <= {"cockpit"}:
                continue
            ov = aabb_overlap(a, b)
            if np.all(ov > min_overlap):
                hits.append(InterferenceHit(a.name, b.name, ov, float(np.prod(ov))))
    return hits
