"""Open differential: equal torque split."""

from __future__ import annotations


def open_split(T_in: float) -> tuple[float, float]:
    """T_L = T_R = T_in / 2."""
    half = 0.5 * T_in
    return half, half
