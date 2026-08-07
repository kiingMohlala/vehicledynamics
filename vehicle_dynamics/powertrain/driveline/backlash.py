"""Gear / driveline backlash dead-zone model."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class BacklashState:
    theta: float = 0.0          # relative angle
    gap: float = 0.0            # rad half-gap
    engaged: bool = False
    torque_scale: float = 0.0   # 0 in deadzone, 1 when meshed
    side: int = 0               # -1, 0, +1


class Backlash:
    """
    Symmetric backlash:
        |θ| < gap  → no torque transmitted (dead zone)
        |θ| ≥ gap  → elastic engagement on excess twist

    Returns the effective engagement angle used by the shaft spring:
        θ_eff = sign(θ) * max(|θ| - gap, 0)
    """

    def __init__(self, gap_rad: float = 0.007, engage_stiffness_scale: float = 1.0):
        self.gap = max(0.0, float(gap_rad))
        self.engage_stiffness_scale = float(engage_stiffness_scale)

    @classmethod
    def from_degrees(cls, gap_deg: float = 0.4, **kw) -> "Backlash":
        return cls(gap_rad=np.deg2rad(gap_deg), **kw)

    def effective_angle(self, theta: float) -> float:
        g = self.gap
        if abs(theta) <= g:
            return 0.0
        return float(np.sign(theta) * (abs(theta) - g))

    def evaluate(self, theta: float) -> BacklashState:
        theta = float(theta)
        if abs(theta) < self.gap:
            return BacklashState(
                theta=theta, gap=self.gap, engaged=False, torque_scale=0.0, side=0
            )
        side = 1 if theta > 0 else -1
        return BacklashState(
            theta=theta,
            gap=self.gap,
            engaged=True,
            torque_scale=self.engage_stiffness_scale,
            side=side,
        )
