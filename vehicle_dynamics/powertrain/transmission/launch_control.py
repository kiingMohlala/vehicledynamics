"""Launch control: hold RPM and modulate clutch."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class LaunchState:
    active: bool = False
    target_rpm: float = 4500.0
    clutch_cmd: float = 0.0
    hold_throttle: float = 0.7


class LaunchControl:
    def __init__(self, target_rpm: float = 4500.0, engage_rate: float = 0.4):
        self.target_rpm = target_rpm
        self.engage_rate = engage_rate
        self.state = LaunchState(target_rpm=target_rpm)

    def enable(self, target_rpm: float | None = None) -> None:
        self.state.active = True
        if target_rpm is not None:
            self.state.target_rpm = target_rpm
        self.state.clutch_cmd = 0.15

    def disable(self) -> None:
        self.state.active = False
        self.state.clutch_cmd = 1.0

    def step(self, rpm: float, dt: float, launch_request: bool) -> LaunchState:
        if not launch_request:
            self.state.active = False
            return self.state
        self.state.active = True
        # Modulate clutch toward lock as RPM near target
        err = (rpm - self.state.target_rpm) / max(self.state.target_rpm, 1.0)
        # If below target, keep clutch light; if at/above, feed in
        if rpm < self.state.target_rpm * 0.95:
            self.state.clutch_cmd = max(0.1, self.state.clutch_cmd - 0.2 * dt)
        else:
            self.state.clutch_cmd = min(1.0, self.state.clutch_cmd + self.engage_rate * dt)
        return self.state
