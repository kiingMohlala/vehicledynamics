"""Pedal-to-request maps."""

from __future__ import annotations

from enum import Enum
import numpy as np


class ThrottleMap(str, Enum):
    LINEAR = "linear"
    PROGRESSIVE = "progressive"
    SPORT = "sport"
    RAIN = "rain"
    ECO = "eco"
    CUSTOM = "custom"


def throttle_factor(pedal: float, map_type: ThrottleMap | str = ThrottleMap.LINEAR) -> float:
    """Map pedal [0,1] → torque demand factor [0,1]."""
    x = float(np.clip(pedal, 0.0, 1.0))
    m = ThrottleMap(map_type) if not isinstance(map_type, ThrottleMap) else map_type
    if m == ThrottleMap.LINEAR:
        return x
    if m == ThrottleMap.PROGRESSIVE:
        return x * x
    if m == ThrottleMap.SPORT:
        # Aggressive early response
        return float(np.clip(1.2 * x ** 0.7, 0.0, 1.0))
    if m == ThrottleMap.RAIN:
        return float(np.clip(0.85 * x ** 1.3, 0.0, 0.9))
    if m == ThrottleMap.ECO:
        return float(np.clip(0.75 * x ** 1.5, 0.0, 0.85))
    # CUSTOM: mild S-curve
    return float(np.clip(0.5 * (np.sin(np.pi * (x - 0.5)) + 1.0), 0.0, 1.0))
