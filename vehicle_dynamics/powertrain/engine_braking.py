"""Closed-throttle engine braking torque."""

from __future__ import annotations

import numpy as np


def engine_brake_torque(
    rpm: float,
    throttle: float,
    *,
    base: float = 30.0,
    rpm_gain: float = 0.012,
    idle_rpm: float = 900.0,
) -> float:
    """
    Negative torque when throttle is low.
    More braking at higher RPM; fades as throttle opens.
    """
    thr = float(np.clip(throttle, 0.0, 1.0))
    if thr > 0.35:
        return 0.0
    fade = (0.35 - thr) / 0.35
    tq = -(base + rpm_gain * max(rpm - idle_rpm, 0.0)) * fade
    return float(tq)
