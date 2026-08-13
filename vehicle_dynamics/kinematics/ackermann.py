"""Ackermann helpers (re-export + parallel/anti-Ackermann)."""
from __future__ import annotations

from .steering_geometry import ackermann_angles, ackermann_percentage, wheel_steer_from_rack


def parallel_steer(steer: float) -> dict:
    return {"inside": float(steer), "outside": float(steer), "ackermann_pct": 0.0}


def anti_ackermann(wheelbase: float, track: float, steer_inside: float, factor: float = -0.5) -> dict:
    ideal = ackermann_angles(wheelbase, track, steer_inside)
    # blend toward opposite of Ackermann
    outside = ideal["inside"] + factor * (ideal["outside"] - ideal["inside"])
    return {"inside": ideal["inside"], "outside": float(outside), "ackermann_pct": float(factor * 100)}
