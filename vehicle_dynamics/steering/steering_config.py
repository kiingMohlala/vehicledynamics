"""Authoritative steering configuration — Phase 14.9.1."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SteeringConfig:
    """Parameters for the front-steering subsystem.

    All angles in radians unless noted.
    """

    max_steer_angle: float = 0.52  # ~30 deg road-wheel limit
    steering_ratio: float = 15.0   # handwheel → road-wheel (for future handwheel input)
    steering_rate: float = 1.2     # rad/s max |dδ/dt| at road wheels
    ackermann_enabled: bool = True
    # Geometry used for Ackermann (must match vehicle)
    wheelbase: float = 2.70
    track_front: float = 1.65
    # Rear steer reserved (always 0 in 14.9.1)
    rear_steer_enabled: bool = False
    max_rear_steer: float = 0.0
