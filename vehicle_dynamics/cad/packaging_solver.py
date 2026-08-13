"""Packaging clearance evaluations."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import numpy as np
from .component import Component
from .interference import detect_interferences, aabb_overlap


@dataclass
class PackagingReport:
    ground_clearance: float
    wheel_to_body_min: float
    engine_to_firewall_ok: bool
    interferences: list
    notes: list[str]


def ground_clearance(components: Iterable[Component]) -> float:
    zmin = min(c.aabb_min[2] for c in components)
    return float(zmin)


def min_distance_between(a: Component, b: Component) -> float:
    """Approximate gap (negative if penetrating)."""
    ov = aabb_overlap(a, b)
    if np.all(ov > 0):
        return float(-np.min(ov))
    # separation along axes
    amin, amax = a.aabb_min, a.aabb_max
    bmin, bmax = b.aabb_min, b.aabb_max
    gaps = []
    for i in range(3):
        if amax[i] < bmin[i]:
            gaps.append(bmin[i] - amax[i])
        elif bmax[i] < amin[i]:
            gaps.append(amin[i] - bmax[i])
        else:
            gaps.append(0.0)
    return float(max(gaps) if gaps else 0.0)


def evaluate_packaging(components: list[Component]) -> PackagingReport:
    by_name = {c.name: c for c in components}
    notes = []
    hits = detect_interferences(components)
    # wheel to body
    wheels = [c for c in components if c.category == "wheel"]
    body = by_name.get("body")
    wheel_gap = 1e9
    if body and wheels:
        for w in wheels:
            wheel_gap = min(wheel_gap, abs(min_distance_between(w, body)))
    else:
        wheel_gap = 0.0
        notes.append("missing body or wheels")

    engine = by_name.get("engine")
    chassis = by_name.get("chassis")
    eng_ok = True
    if engine and chassis:
        # soft check: centers not coincident with full overlap volume huge
        eng_ok = not any(h.a in ("engine", "chassis") and h.b in ("engine", "chassis") for h in hits)

    gc = ground_clearance(components)
    return PackagingReport(
        ground_clearance=gc,
        wheel_to_body_min=float(wheel_gap if wheel_gap < 1e8 else 0.0),
        engine_to_firewall_ok=eng_ok,
        interferences=hits,
        notes=notes,
    )
