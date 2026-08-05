"""Viscous limited-slip differential."""

from __future__ import annotations

from .open_diff import open_split


def viscous_split(
    T_in: float,
    omega_L: float,
    omega_R: float,
    *,
    k_v: float = 8.0,
) -> tuple[float, float, float]:
    """
    T_transfer = k_v (ω_L - ω_R)
    Applied from faster to slower.
    """
    T_L0, T_R0 = open_split(T_in)
    T_tr = k_v * (omega_L - omega_R)
    # Positive T_tr: left faster → subtract from L, add to R
    T_L = T_L0 - 0.5 * T_tr
    T_R = T_R0 + 0.5 * T_tr
    return float(T_L), float(T_R), float(abs(T_tr))
