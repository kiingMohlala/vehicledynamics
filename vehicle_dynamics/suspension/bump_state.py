"""Bump-steer state containers."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class BumpSteerParams:
    """
    Linear bump-steer coefficients [rad / m of wheel travel].

    Positive gain: positive wheel bump (compression) → positive toe change
    (sign convention follows geometry_state toe: same as static toe).

    gain = 0 → Phase 6.2 behaviour (regression).
    """
    gain_fl: float = 0.0
    gain_fr: float = 0.0
    gain_rl: float = 0.0
    gain_rr: float = 0.0

    @classmethod
    def neutral(cls) -> "BumpSteerParams":
        return cls()

    @classmethod
    def symmetric(cls, front: float = 0.0, rear: float = 0.0) -> "BumpSteerParams":
        return cls(gain_fl=front, gain_fr=front, gain_rl=rear, gain_rr=rear)

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.gain_fl, self.gain_fr, self.gain_rl, self.gain_rr], dtype=float
        )


@dataclass
class BumpSteerState:
    """Per-wheel bump-steer snapshot."""
    wheel_travel: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # [m] +bump = compression
    toe_bump: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # [rad]
    toe_static: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # [rad]
    toe_total: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # static + bump

    def diagnostics(self) -> dict:
        return {
            "wheel_travel_m": self.wheel_travel.tolist(),
            "toe_bump_rad": self.toe_bump.tolist(),
            "toe_static_rad": self.toe_static.tolist(),
            "toe_total_rad": self.toe_total.tolist(),
            "toe_bump_deg": np.degrees(self.toe_bump).tolist(),
            "toe_total_deg": np.degrees(self.toe_total).tolist(),
        }
