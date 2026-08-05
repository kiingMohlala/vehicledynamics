"""Yaw moment request aggregator (brake + TV)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .sensor_model import SensorReading


@dataclass
class YawController:
    enabled: bool = True
    kp: float = 200.0
    kd: float = 20.0
    max_moment: float = 1500.0
    _prev_e: float = 0.0

    def step(
        self,
        sensors: SensorReading,
        r_ref: float,
        dt: float,
    ) -> tuple[float, float]:
        """
        Returns (tv_request N·m, yaw_error).
        Positive TV → more right torque → positive yaw (CCW if convention matches).
        """
        if not self.enabled:
            return 0.0, 0.0
        e = r_ref - sensors.yaw_rate
        de = (e - self._prev_e) / max(dt, 1e-4)
        self._prev_e = e
        m = self.kp * e + self.kd * de
        m = float(np.clip(m, -self.max_moment, self.max_moment))
        # Map moment to TV delta (approx track/2)
        tv = m * 0.3
        return tv, float(e)
