"""Anti-dive / anti-squat from side-view instant centers."""
from __future__ import annotations

import numpy as np


def anti_dive(
    ic_side_xz: np.ndarray,
    contact_xz: np.ndarray,
    cg_height: float,
    wheelbase: float,
    brake_bias_front: float = 0.6,
) -> float:
    """
    Front anti-dive percentage.
    Roughly: 100 * (IC height geometry) / (CG pitch geometry).
    """
    ic = np.asarray(ic_side_xz, dtype=float)
    cp = np.asarray(contact_xz, dtype=float)
    dx = ic[0] - cp[0]
    if abs(dx) < 1e-9 or abs(wheelbase) < 1e-9 or cg_height < 1e-9:
        return 0.0
    # swing arm angle
    theta = np.arctan2(ic[1] - cp[1], abs(dx))
    # standard approximation
    anti = 100.0 * (np.tan(theta) * wheelbase / cg_height) * brake_bias_front
    return float(np.clip(anti, -200, 200))


def anti_squat(
    ic_side_xz: np.ndarray,
    contact_xz: np.ndarray,
    cg_height: float,
    wheelbase: float,
    drive_bias_rear: float = 1.0,
) -> float:
    ic = np.asarray(ic_side_xz, dtype=float)
    cp = np.asarray(contact_xz, dtype=float)
    dx = ic[0] - cp[0]
    if abs(dx) < 1e-9 or abs(wheelbase) < 1e-9 or cg_height < 1e-9:
        return 0.0
    theta = np.arctan2(ic[1] - cp[1], abs(dx))
    anti = 100.0 * (np.tan(theta) * wheelbase / cg_height) * drive_bias_rear
    return float(np.clip(anti, -200, 200))


def anti_lift(ic_side_xz, contact_xz, cg_height, wheelbase, brake_bias_rear=0.4) -> float:
    return anti_dive(ic_side_xz, contact_xz, cg_height, wheelbase, brake_bias_front=brake_bias_rear)
