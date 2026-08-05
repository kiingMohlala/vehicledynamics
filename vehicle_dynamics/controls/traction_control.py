"""Traction control: limit wheelspin via torque cut and optional brake."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class TractionControl:
    slip_target: float = 0.12
    slip_max: float = 0.25
    kp: float = 2.0
    enabled: bool = True

    def step(
        self,
        sensors: SensorReading,
        throttle: float,
        dt: float,
    ) -> tuple[float, float, bool]:
        """
        Returns (engine_torque_limit 0..1, brake_spin_nudge 0..1, active).
        """
        if not self.enabled or throttle < 0.05:
            return 1.0, 0.0, False

        # Driven wheels assumed rear (2,3) for RWD baseline; use max of all
        slip = float(np.max(np.abs(sensors.slip_ratio)))
        if slip <= self.slip_target:
            return 1.0, 0.0, False

        over = slip - self.slip_target
        cut = float(np.clip(1.0 - self.kp * over, 0.15, 1.0))
        brake_nudge = 0.0
        if slip > self.slip_max:
            brake_nudge = float(np.clip((slip - self.slip_max) * 0.5, 0.0, 0.3))
        return cut, brake_nudge, True
