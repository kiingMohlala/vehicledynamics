"""Hill-hold: retain brake pressure on gradient until throttle."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class HillHold:
    enabled: bool = True
    ax_threshold: float = 0.5   # m/s² grade proxy when nearly stopped
    release_throttle: float = 0.08
    hold_pressure: float = 0.35
    _holding: bool = False

    def step(
        self,
        sensors: SensorReading,
        driver_brake: float,
        throttle: float,
        request: bool,
    ) -> tuple[np.ndarray, bool]:
        """Returns brake pressures, active."""
        if not self.enabled:
            return np.ones(4) * driver_brake, False

        stopped = abs(sensors.vx) < 0.5
        if request and stopped and driver_brake > 0.1:
            self._holding = True
        if throttle > self.release_throttle or abs(sensors.vx) > 1.5:
            self._holding = False

        if self._holding:
            p = max(driver_brake, self.hold_pressure)
            return np.ones(4) * p, True
        return np.ones(4) * driver_brake, False
