"""Build final torque demand factor for powertrain."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class TorqueRequestBuilder:
    max_wheel_torque: float = 4000.0

    def build(
        self,
        torque_factor: float,
        max_mode_factor: float = 1.0,
        launch_scale: float = 1.0,
        pit_scale: float = 1.0,
        cruise_throttle: float = 0.0,
        driver_throttle_active: bool = True,
    ) -> tuple[float, float]:
        """
        Returns (torque_factor_out, wheel_torque_request).
        Cruise can inject throttle when driver not pressing pedal.
        """
        base = torque_factor if driver_throttle_active else cruise_throttle
        f = float(np.clip(base * max_mode_factor * launch_scale * pit_scale, 0.0, 1.0))
        return f, f * self.max_wheel_torque
