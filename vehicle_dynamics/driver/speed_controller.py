"""Longitudinal speed PID."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SpeedController:
    kp: float = 0.15
    ki: float = 0.02
    kd: float = 0.01
    max_throttle: float = 1.0
    max_brake: float = 1.0
    _integ: float = 0.0
    _prev: float = 0.0

    def step(self, v: float, v_ref: float, dt: float) -> tuple[float, float]:
        e = v_ref - v
        self._integ += e * dt
        self._integ = float(np.clip(self._integ, -20.0, 20.0))
        d = (e - self._prev) / max(dt, 1e-4)
        self._prev = e
        u = self.kp * e + self.ki * self._integ + self.kd * d
        if u >= 0:
            thr = float(np.clip(u, 0.0, self.max_throttle))
            br = 0.0
        else:
            thr = 0.0
            br = float(np.clip(-u, 0.0, self.max_brake))
        return thr, br
