"""
Phase 14.9.2 — Wheel-local velocity and slip angles.

Contact-patch kinematics between steering angles and Dugoff.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class WheelSlipState:
    delta: float
    vx_c: float   # contact velocity in vehicle frame
    vy_c: float
    vx_t: float   # tire-frame longitudinal
    vy_t: float   # tire-frame lateral
    alpha: float  # slip angle (rad)
    kappa: float  # longitudinal slip (filled by caller)


def compute_wheel_slip_angles(
    *,
    vx: float,
    vy: float,
    yaw_rate: float,
    deltas: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    v_eps: float = 0.5,
) -> list[WheelSlipState]:
    """
    Per-wheel contact kinematics.

    Sign convention (ISO-ish for this plant):
      +δ = left turn (CCW from above)
      +α = tire needs lateral force to the left in vehicle sense under +δ
      α = atan2(-vy_t, |vx_t|) so left-steer at pure roll → α > 0

    Low-speed: |vx_t| floored at v_eps to avoid NaN.
    """
    out: list[WheelSlipState] = []
    for i in range(4):
        d = float(deltas[i])
        vx_c = float(vx - yaw_rate * ys[i])
        vy_c = float(vy + yaw_rate * xs[i])
        c, s = np.cos(d), np.sin(d)
        vx_t = float(c * vx_c + s * vy_c)
        vy_t = float(-s * vx_c + c * vy_c)
        # Slip angle: velocity relative to wheel heading
        alpha = float(np.arctan2(-vy_t, max(abs(vx_t), v_eps)))
        out.append(WheelSlipState(
            delta=d, vx_c=vx_c, vy_c=vy_c, vx_t=vx_t, vy_t=vy_t,
            alpha=alpha, kappa=0.0,
        ))
    return out
