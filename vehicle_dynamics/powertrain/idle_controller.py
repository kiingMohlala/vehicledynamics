"""Idle RPM hold / anti-stall assist."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class IdleController:
    idle_rpm: float = 900.0
    kp: float = 0.0015
    ki: float = 0.0008
    max_assist: float = 0.25
    enable_rpm: float = 1400.0  # only assist below this when pedal low

    def __post_init__(self) -> None:
        self._integ = 0.0

    def reset(self) -> None:
        self._integ = 0.0

    def assist(self, rpm: float, pedal: float, dt: float) -> float:
        """Extra throttle command 0..max_assist when near idle."""
        if pedal > 0.05 or rpm > self.enable_rpm:
            self._integ *= 0.9
            return 0.0
        err = self.idle_rpm - rpm
        self._integ = float(np.clip(self._integ + err * dt, -500.0, 500.0))
        u = self.kp * err + self.ki * self._integ
        return float(np.clip(u, 0.0, self.max_assist))
