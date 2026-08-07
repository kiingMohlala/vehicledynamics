"""Closed-loop cruise speed hold."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class CruiseControl:
    enabled: bool = True
    kp: float = 0.15
    ki: float = 0.02
    max_throttle: float = 0.7
    max_brake: float = 0.4

    def __post_init__(self) -> None:
        self._integral = 0.0
        self._active = False
        self._set = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._active = False

    def step(
        self,
        active: bool,
        set_speed: float,
        vehicle_speed: float,
        dt: float,
    ) -> dict:
        if not self.enabled or not active or set_speed <= 0:
            self._active = False
            self._integral = 0.0
            return {"active": False, "throttle": 0.0, "brake": 0.0}

        self._active = True
        self._set = set_speed
        err = set_speed - vehicle_speed
        self._integral = float(np.clip(self._integral + err * dt, -10.0, 10.0))
        u = self.kp * err + self.ki * self._integral
        thr = float(np.clip(u, 0.0, self.max_throttle))
        brk = float(np.clip(-u, 0.0, self.max_brake)) if err < -1.0 else 0.0
        return {"active": True, "throttle": thr, "brake": brk, "error": err}
