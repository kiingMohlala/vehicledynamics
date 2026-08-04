"""Jacking force state containers (Phase 6.7)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class JackingParams:
    """
    First-order geometric load transfer (jacking).

    ΔFz_pair ≈ Fy_axle * h_RC / track

    enabled=False → zero jacking (Phase 6.6 regression).
    """
    enabled: bool = True
    track_f: float = 1.55
    track_r: float = 1.55
    Fz_min: float = 50.0


@dataclass
class JackingState:
    """Per-axle jacking snapshot."""
    rc_front: float = 0.0
    rc_rear: float = 0.0
    Fy_front: float = 0.0   # sum of front lateral forces (body frame)
    Fy_rear: float = 0.0
    dFz_front: float = 0.0  # magnitude transferred front axle (pair)
    dFz_rear: float = 0.0
    # applied to wheels [FL, FR, RL, RR] — signed add to Fz
    dFz_wheels: np.ndarray = field(default_factory=lambda: np.zeros(4))

    def diagnostics(self) -> dict:
        return {
            "rc_front_m": self.rc_front,
            "rc_rear_m": self.rc_rear,
            "Fy_front": self.Fy_front,
            "Fy_rear": self.Fy_rear,
            "dFz_front": self.dFz_front,
            "dFz_rear": self.dFz_rear,
            "dFz_wheels": self.dFz_wheels.tolist(),
        }
