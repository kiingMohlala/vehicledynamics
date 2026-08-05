"""Electronic Brake Distribution (EBD)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class EBDController:
    enabled: bool = True
    base_front: float = 0.65
    ax_gain: float = 0.08

    def step(
        self,
        sensors: SensorReading,
        driver_brake: float,
    ) -> tuple[np.ndarray, bool]:
        """
        Front/rear bias from deceleration. Returns pressures[4], active.
        """
        if not self.enabled or driver_brake < 0.02:
            p = np.ones(4) * driver_brake
            return p, False

        # More front bias under hard deceleration (weight transfer)
        ax = min(0.0, sensors.ax)  # braking ax < 0
        front = float(np.clip(self.base_front - self.ax_gain * ax, 0.55, 0.80))
        rear = 1.0 - front
        p = np.array([
            driver_brake * front,
            driver_brake * front,
            driver_brake * rear,
            driver_brake * rear,
        ])
        active = abs(front - self.base_front) > 0.02
        return p, active
