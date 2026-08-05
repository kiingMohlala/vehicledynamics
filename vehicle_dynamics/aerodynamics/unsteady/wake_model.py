"""Multi-vehicle wake field."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .drafting import DraftingParams, drafting_factors


@dataclass
class WakeSource:
    """Lead car pose in same frame as ego (x forward)."""

    x: float
    y: float = 0.0
    heading: float = 0.0
    strength: float = 1.0   # 0..1 scale


@dataclass
class WakeField:
    sources: list[WakeSource] = field(default_factory=list)
    params: DraftingParams = field(default_factory=DraftingParams)


def _gap_along_wake(ego_x: float, ego_y: float, src: WakeSource) -> float | None:
    """
    Longitudinal gap if ego is behind source within a lateral corridor.
    Returns None if not in wake.
    """
    # Vector from source to ego
    dx = ego_x - src.x
    dy = ego_y - src.y
    c, s = np.cos(src.heading), np.sin(src.heading)
    # Source body frame: x along heading
    lon = c * dx + s * dy
    lat = -s * dx + c * dy
    if lon < 0:
        return None  # ego is ahead
    # Lateral wake width ~ grows slowly
    half_width = 1.5 + 0.05 * lon
    if abs(lat) > half_width:
        return None
    return float(lon)


def evaluate_wake(
    field: WakeField,
    ego_x: float = 0.0,
    ego_y: float = 0.0,
) -> dict[str, float]:
    """
    Combine wake effects from all sources (take strongest draft).
    """
    best = {"Cd_factor": 1.0, "Cl_factor": 1.0, "cooling_factor": 1.0, "wake_strength": 0.0}
    for src in field.sources:
        gap = _gap_along_wake(ego_x, ego_y, src)
        if gap is None:
            continue
        fac = drafting_factors(gap, field.params)
        # Scale by source strength
        w = src.strength * fac["wake_strength"]
        Cd_f = 1.0 - (1.0 - fac["Cd_factor"]) * src.strength
        Cl_f = 1.0 - (1.0 - fac["Cl_factor"]) * src.strength
        cool = 1.0 - (1.0 - fac["cooling_factor"]) * src.strength
        if w > best["wake_strength"]:
            best = {
                "Cd_factor": Cd_f,
                "Cl_factor": Cl_f,
                "cooling_factor": cool,
                "wake_strength": w,
                "gap": gap,
            }
    return best
