"""Automatic shift decision logic."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ShiftStrategy:
    aggressiveness: float = 0.5
    rpm_up_base: float = 2800.0
    rpm_down_base: float = 1400.0
    rpm_redline: float = 7000.0
    kickdown_threshold: float = 0.85

    def rpm_up(self) -> float:
        # Higher aggressiveness → shift later
        return float(np.clip(
            self.rpm_up_base + self.aggressiveness * 2500.0,
            2000.0, self.rpm_redline - 200.0,
        ))

    def rpm_down(self) -> float:
        return float(np.clip(
            self.rpm_down_base + self.aggressiveness * 800.0,
            800.0, 3000.0,
        ))

    def decide(
        self,
        rpm: float,
        gear: int,
        throttle: float,
        n_gears: int = 6,
    ) -> int:
        """Returns shift request: +1 up, -1 down, 0 hold."""
        if gear < 1:
            return 0
        if throttle >= self.kickdown_threshold and rpm < self.rpm_redline - 500:
            if gear > 1 and rpm < self.rpm_up():
                return -1  # kickdown
        if rpm >= self.rpm_up() and gear < n_gears:
            return 1
        if rpm <= self.rpm_down() and gear > 1:
            return -1
        return 0
