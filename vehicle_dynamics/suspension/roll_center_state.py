"""Roll-center migration state (Phase 6.5)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class RollCenterState:
    """Front/rear roll-center snapshot (diagnostic only)."""
    wheel_travel: np.ndarray = field(default_factory=lambda: np.zeros(4))
    rc_front: float = 0.0          # [m] height above ground
    rc_rear: float = 0.0
    rc_front_static: float = 0.0   # design (z=0) baseline
    rc_rear_static: float = 0.0
    rc_front_migration: float = 0.0  # rc_front - rc_front_static
    rc_rear_migration: float = 0.0
    # optional per-corner IC diagnostics
    ic_y: np.ndarray = field(default_factory=lambda: np.full(4, np.nan))
    ic_z: np.ndarray = field(default_factory=lambda: np.full(4, np.nan))

    def diagnostics(self) -> dict:
        return {
            "wheel_travel_m": self.wheel_travel.tolist(),
            "rc_front_m": self.rc_front,
            "rc_rear_m": self.rc_rear,
            "rc_front_static_m": self.rc_front_static,
            "rc_rear_static_m": self.rc_rear_static,
            "rc_front_migration_m": self.rc_front_migration,
            "rc_rear_migration_m": self.rc_rear_migration,
            "ic_y_m": self.ic_y.tolist(),
            "ic_z_m": self.ic_z.tolist(),
        }
