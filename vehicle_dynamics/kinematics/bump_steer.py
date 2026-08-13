"""Bump-steer curve from toe vs wheel travel."""
from __future__ import annotations

import numpy as np


def bump_steer_gradient(travels: np.ndarray, toes: np.ndarray) -> float:
    """Linear fit d(toe)/d(travel) in rad/m near zero travel."""
    travels = np.asarray(travels, dtype=float)
    toes = np.asarray(toes, dtype=float)
    if len(travels) < 2:
        return 0.0
    # use central points
    coef = np.polyfit(travels, toes, 1)
    return float(coef[0])


def bump_steer_curve(travels: np.ndarray, toes: np.ndarray) -> dict:
    return {
        "travel": np.asarray(travels, dtype=float),
        "toe": np.asarray(toes, dtype=float),
        "gradient_rad_per_m": bump_steer_gradient(travels, toes),
        "gradient_deg_per_mm": float(np.degrees(bump_steer_gradient(travels, toes)) / 1000.0),
    }
