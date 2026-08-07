"""Motorsport pit-lane speed limiter."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class PitLimiter:
    enabled: bool = True
    pit_speed_mps: float = 16.67   # ~60 km/h
    kp: float = 0.4

    def step(self, active: bool, vehicle_speed: float) -> dict:
        if not self.enabled or not active:
            return {"active": False, "torque_scale": 1.0, "overspeed": False}
        over = vehicle_speed > self.pit_speed_mps
        if not over:
            # Soft approach: scale down as we near limit
            ratio = vehicle_speed / max(self.pit_speed_mps, 0.1)
            scale = float(np.clip(1.2 - ratio, 0.0, 1.0))
            return {"active": True, "torque_scale": scale, "overspeed": False}
        # Cut hard
        return {"active": True, "torque_scale": 0.0, "overspeed": True}
