"""Clean output interface toward differential / wheels."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DrivetrainOutput:
    wheel_torque: float = 0.0      # N·m total axle (pre-diff split)
    gearbox_rpm: float = 0.0
    clutch_slip: float = 0.0       # rad/s
    current_gear: int = 0
    shift_active: bool = False
