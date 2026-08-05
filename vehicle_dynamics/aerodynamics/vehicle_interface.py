"""Map vehicle state → RideHeightState for closed-loop aero."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .ride_height import RideHeightState
from .coefficients import AeroConfig


@dataclass
class VehicleAeroInput:
    """Minimal vehicle state needed by the aero closed loop."""

    speed: float                    # m/s (longitudinal)
    yaw_rate: float = 0.0           # rad/s
    sideslip: float = 0.0           # rad
    pitch: float = 0.0              # rad, nose-up positive
    heave: float = 0.0              # m, vertical CG displacement (up +)
    yaw_angle: float = 0.0          # rad (for crosswind / body yaw)
    steer: float = 0.0


def ride_from_pitch_heave(
    pitch: float,
    heave: float,
    cfg: AeroConfig,
    *,
    h_front0: float | None = None,
    h_rear0: float | None = None,
) -> RideHeightState:
    """
    Kinematic ride heights from small-angle pitch about mid-wheelbase.

    Convention: pitch > 0 = nose-up.
      front body rises → h_front increases
      rear body drops  → h_rear decreases

    h_f = h_f0 + heave + a * pitch
    h_r = h_r0 + heave - b * pitch
    """
    hf0 = cfg.h_front_ref if h_front0 is None else h_front0
    hr0 = cfg.h_rear_ref if h_rear0 is None else h_rear0
    a = 0.5 * cfg.wheelbase
    b = 0.5 * cfg.wheelbase
    hf = hf0 + heave + a * pitch
    hr = hr0 + heave - b * pitch
    hf = max(hf, 0.005)
    hr = max(hr, 0.005)
    return RideHeightState(h_front=hf, h_rear=hr, pitch_rad=pitch, yaw_rad=0.0)


def ride_from_vehicle_input(
    inp: VehicleAeroInput,
    cfg: AeroConfig,
) -> RideHeightState:
    ride = ride_from_pitch_heave(inp.pitch, inp.heave, cfg)
    ride.yaw_rad = inp.yaw_angle
    return ride
