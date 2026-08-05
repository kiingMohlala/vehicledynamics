"""Steering controllers: PID, Pure Pursuit, Stanley."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SteeringPID:
    kp: float = 0.8
    ki: float = 0.05
    kd: float = 0.1
    max_steer: float = 0.6
    _integ: float = 0.0
    _prev: float = 0.0

    def step(self, heading_error: float, dt: float) -> float:
        self._integ += heading_error * dt
        self._integ = float(np.clip(self._integ, -1.0, 1.0))
        d = (heading_error - self._prev) / max(dt, 1e-4)
        self._prev = heading_error
        u = self.kp * heading_error + self.ki * self._integ + self.kd * d
        return float(np.clip(u, -self.max_steer, self.max_steer))


@dataclass
class PurePursuit:
    lookahead: float = 8.0
    max_steer: float = 0.6
    wheelbase: float = 2.7

    def step(
        self,
        x: float,
        y: float,
        psi: float,
        target_x: float,
        target_y: float,
    ) -> float:
        # Transform target into vehicle frame
        dx = target_x - x
        dy = target_y - y
        local_x = np.cos(psi) * dx + np.sin(psi) * dy
        local_y = -np.sin(psi) * dx + np.cos(psi) * dy
        Ld = max(np.hypot(local_x, local_y), 0.5)
        # α ≈ local lateral / Ld
        curvature = 2.0 * local_y / (Ld * Ld)
        delta = np.arctan(self.wheelbase * curvature)
        return float(np.clip(delta, -self.max_steer, self.max_steer))


@dataclass
class StanleyController:
    k: float = 1.2
    max_steer: float = 0.6
    eps: float = 1.0

    def step(
        self,
        heading_error: float,
        cross_track: float,
        speed: float,
    ) -> float:
        v = max(abs(speed), self.eps)
        # cross_track > 0 ⇒ vehicle left of path ⇒ steer right (negative)
        delta = heading_error - np.arctan2(self.k * cross_track, v)
        return float(np.clip(delta, -self.max_steer, self.max_steer))
