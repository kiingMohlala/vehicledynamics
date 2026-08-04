"""Camber-gain state containers (Phase 6.4)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class CamberGainParams:
    """
    Linear camber-gain coefficients [rad / m of wheel travel].

    Positive gain: positive wheel bump (compression) → positive camber change
    (same sign convention as geometry_state camber).

    gain = 0 → Phase 6.3 behaviour (regression).
    """
    gain_fl: float = 0.0
    gain_fr: float = 0.0
    gain_rl: float = 0.0
    gain_rr: float = 0.0

    @classmethod
    def neutral(cls) -> "CamberGainParams":
        return cls()

    @classmethod
    def symmetric(cls, front: float = 0.0, rear: float = 0.0) -> "CamberGainParams":
        return cls(gain_fl=front, gain_fr=front, gain_rl=rear, gain_rr=rear)

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.gain_fl, self.gain_fr, self.gain_rl, self.gain_rr], dtype=float
        )


@dataclass
class CamberState:
    """Per-wheel camber snapshot (diagnostic — does not drive tire forces)."""
    wheel_travel: np.ndarray = field(default_factory=lambda: np.zeros(4))
    camber_static: np.ndarray = field(default_factory=lambda: np.zeros(4))
    camber_gain: np.ndarray = field(default_factory=lambda: np.zeros(4))
    camber_total: np.ndarray = field(default_factory=lambda: np.zeros(4))

    def diagnostics(self) -> dict:
        return {
            "wheel_travel_m": self.wheel_travel.tolist(),
            "camber_static_rad": self.camber_static.tolist(),
            "camber_gain_rad": self.camber_gain.tolist(),
            "camber_total_rad": self.camber_total.tolist(),
            "camber_static_deg": np.degrees(self.camber_static).tolist(),
            "camber_gain_deg": np.degrees(self.camber_gain).tolist(),
            "camber_total_deg": np.degrees(self.camber_total).tolist(),
        }
