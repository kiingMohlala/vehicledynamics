"""Roll-center migration state (Phase 6.5)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class RollCenterState:
    """Front/rear roll-center snapshot (diagnostic only)."""
    wheel_travel: np.ndarray = field(default_factory=lambda: np.zeros(4))
    rc_front_z: float = 0.0
    rc_rear_z: float = 0.0
    rc_front_static_z: float = 0.0
    rc_rear_static_z: float = 0.0
    rc_front_migration: float = 0.0  # current - static
    rc_rear_migration: float = 0.0
    ic_front_y: float = float("nan")
    ic_front_z: float = float("nan")
    ic_rear_y: float = float("nan")
    ic_rear_z: float = float("nan")

    def diagnostics(self) -> dict:
        return {
            "wheel_travel_m": self.wheel_travel.tolist(),
            "rc_front_z": self.rc_front_z,
            "rc_rear_z": self.rc_rear_z,
            "rc_front_static_z": self.rc_front_static_z,
            "rc_rear_static_z": self.rc_rear_static_z,
            "rc_front_migration": self.rc_front_migration,
            "rc_rear_migration": self.rc_rear_migration,
            "ic_front_yz": [self.ic_front_y, self.ic_front_z],
            "ic_rear_yz": [self.ic_rear_y, self.ic_rear_z],
        }
