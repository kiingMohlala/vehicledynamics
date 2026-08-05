"""
Phase 5.1 – Ackermann steering geometry.

Given a single handwheel / average steer command δ, compute independent
front-left and front-right road-wheel angles so the wheel axes intersect
at a common instantaneous centre on the rear-axle line (classical Ackermann).

Sign convention (matches dual-track / bicycle):
  +δ  → left turn
  +y  → left
  Inside wheel (left when δ > 0) receives the larger |steer angle|.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SteeringParameters:
    """Ackermann / steering configuration."""
    use_ackermann: bool = True
    # If False, both front wheels receive the same clipped δ (Phase 5.0 behaviour).
    # Geometry is taken from DualTrackParameters (a, b, track_f).


def ackermann_angles(
    delta: float,
    wheelbase: float,
    track_f: float,
    delta_max: float = 0.6,
) -> tuple[float, float]:
    """
    Map a centreline steer command to (delta_fl, delta_fr).

    Parameters
    ----------
    delta : float
        Centreline / handwheel-equivalent steer [rad], positive = left.
    wheelbase : float
        L = a + b [m].
    track_f : float
        Front track width [m].
    delta_max : float
        Absolute limit applied to both outputs [rad].

    Returns
    -------
    delta_fl, delta_fr : float
        Road-wheel steer angles [rad].
    """
    delta = float(np.clip(delta, -delta_max, delta_max))

    if abs(delta) < 1e-12:
        return 0.0, 0.0

    # Distance from vehicle centreline to instantaneous centre (rear-axle line).
    # R has the same sign as delta: positive R → IC to the left of centreline.
    R = wheelbase / np.tan(delta)

    # Left wheel is toward +y; for left turn (R > 0) it is the inside wheel.
    # δ_i = atan(L / (R - y_i)) with y_left = +track/2, y_right = -track/2
    # Equivalent:
    half_t = 0.5 * track_f
    delta_fl = float(np.arctan(wheelbase / (R - half_t)))
    delta_fr = float(np.arctan(wheelbase / (R + half_t)))

    delta_fl = float(np.clip(delta_fl, -delta_max, delta_max))
    delta_fr = float(np.clip(delta_fr, -delta_max, delta_max))
    return delta_fl, delta_fr


def equal_angles(delta: float, delta_max: float = 0.6) -> tuple[float, float]:
    """Phase 5.0 parallel steer: both front wheels get the same angle."""
    d = float(np.clip(delta, -delta_max, delta_max))
    return d, d


def front_steer_angles(
    delta: float,
    wheelbase: float,
    track_f: float,
    delta_max: float = 0.6,
    use_ackermann: bool = True,
) -> tuple[float, float]:
    """Dispatch to Ackermann or equal-steer."""
    if use_ackermann:
        return ackermann_angles(delta, wheelbase, track_f, delta_max)
    return equal_angles(delta, delta_max)


def ideal_ackermann_relation(
    delta_fl: float,
    delta_fr: float,
    wheelbase: float,
    track_f: float,
) -> float:
    """
    Residual of the classic identity:
        cot(δ_out) - cot(δ_in) = track / wheelbase
    Returns absolute residual (0 = perfect Ackermann).
    """
    # Determine inside/outside by magnitude
    if abs(delta_fl) >= abs(delta_fr):
        d_in, d_out = delta_fl, delta_fr
    else:
        d_in, d_out = delta_fr, delta_fl

    if abs(d_in) < 1e-9 or abs(d_out) < 1e-9:
        return 0.0 if abs(d_in) < 1e-9 and abs(d_out) < 1e-9 else 1e6

    # Preserve signs: cot is odd
    residual = (1.0 / np.tan(d_out) - 1.0 / np.tan(d_in)) - (track_f / wheelbase) * np.sign(d_in)
    # For opposite-sign pathological cases, just return large residual
    if d_in * d_out < 0:
        return abs(residual) + 10.0
    return float(abs(residual))
