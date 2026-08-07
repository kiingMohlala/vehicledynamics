"""Steering geometry: rack displacement → road-wheel angles."""
from __future__ import annotations

import numpy as np


def wheel_steer_from_rack(
    rack_travel: float,
    tierod_length: float,
    steering_arm_length: float,
    rack_y: float = 0.30,
) -> float:
    """
    Approximate planar steering: rack moves laterally, tierod pushes steering arm.
    Returns road-wheel steer angle (rad).
    """
    # small-angle / geometric approx
    if steering_arm_length < 1e-9:
        return 0.0
    # lateral displacement at arm outer ≈ rack_travel * (lever ratio)
    ratio = tierod_length / (tierod_length + 1e-9)
    delta = np.arcsin(np.clip(rack_travel * ratio / steering_arm_length, -1.0, 1.0))
    return float(delta)


def ackermann_angles(wheelbase: float, track: float, steer_inside: float) -> dict:
    """
    Given inside wheel angle, compute ideal outside angle for Ackermann.
    cot(δ_o) - cot(δ_i) = track / wheelbase
    """
    di = float(steer_inside)
    if abs(di) < 1e-12:
        return {"inside": 0.0, "outside": 0.0, "ackermann_pct": 100.0}
    # cot(δ_out) - cot(δ_in) = track/wheelbase  => |δ_in| > |δ_out|
    cot_i = 1.0 / np.tan(di)
    cot_o = cot_i + np.sign(di) * track / max(wheelbase, 1e-9)
    do = float(np.arctan(1.0 / cot_o)) if abs(cot_o) > 1e-9 else 0.0
    do = abs(do) * np.sign(di)
    return {"inside": di, "outside": do, "ackermann_pct": 100.0}


def ackermann_percentage(delta_in: float, delta_out: float, wheelbase: float, track: float) -> float:
    """Compare actual outer angle to ideal Ackermann outer angle."""
    ideal = ackermann_angles(wheelbase, track, delta_in)["outside"]
    if abs(ideal) < 1e-9:
        return 100.0
    return float(100.0 * delta_out / ideal)
