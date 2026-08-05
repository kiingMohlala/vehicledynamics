"""Driver internal state."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DriverState:
    time: float = 0.0
    s_path: float = 0.0              # arc length progress
    cross_track: float = 0.0
    heading_error: float = 0.0
    speed_error: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    target_speed: float = 0.0
    mode: str = "idle"
