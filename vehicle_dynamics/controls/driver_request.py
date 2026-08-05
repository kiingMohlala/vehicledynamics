"""Driver demand inputs."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DriverInputs:
    throttle: float = 0.0          # 0..1
    brake: float = 0.0             # 0..1 pedal
    steer: float = 0.0             # rad
    clutch: float = 1.0            # 1 engaged
    gear_request: int = 0
    launch_request: bool = False
    hill_hold_request: bool = False
