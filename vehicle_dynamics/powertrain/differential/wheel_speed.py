"""Wheel / axle speed helpers for tire coupling."""

from __future__ import annotations


def axle_speed(omega_L: float, omega_R: float) -> float:
    """Carrier speed ω_c = (ω_L + ω_R) / 2."""
    return 0.5 * (omega_L + omega_R)


def differential_speed(omega_L: float, omega_R: float) -> float:
    """Relative speed Δω = ω_L - ω_R."""
    return omega_L - omega_R
