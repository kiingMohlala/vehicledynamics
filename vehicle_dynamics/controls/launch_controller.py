"""Launch control: hold RPM band and manage throttle/clutch commands."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class LaunchController:
    enabled: bool = True
    target_rpm: float = 4500.0
    rpm_band: float = 400.0
    slip_target: float = 0.15

    def step(
        self,
        sensors: SensorReading,
        launch_request: bool,
        throttle: float,
        dt: float,
    ) -> tuple[float, float, bool]:
        """
        Returns (throttle_cmd, clutch_cmd, active).
        """
        if not self.enabled or not launch_request:
            return throttle, 1.0, False

        rpm = sensors.engine_rpm
        slip = float(np.max(np.abs(sensors.slip_ratio[2:])))  # rear

        # Throttle to hold RPM near target
        if rpm < self.target_rpm - self.rpm_band:
            thr = min(1.0, throttle + 0.3)
        elif rpm > self.target_rpm + self.rpm_band:
            thr = max(0.3, throttle - 0.2)
        else:
            thr = float(np.clip(throttle, 0.4, 0.9))

        # Clutch: feed in if slip near target
        if slip > self.slip_target + 0.1:
            clutch = 0.25
        elif slip > self.slip_target:
            clutch = 0.5
        else:
            clutch = min(1.0, 0.4 + 0.5 * (self.slip_target - slip) / max(self.slip_target, 0.05))

        return float(thr), float(np.clip(clutch, 0.1, 1.0)), True
