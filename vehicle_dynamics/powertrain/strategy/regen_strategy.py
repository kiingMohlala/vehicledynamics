"""Regen demand from brake pedal + mode."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class RegenStrategy:
    def request(
        self,
        brake: float,
        regen_level: float,
        vehicle_speed: float,
        fade_speed: float = 3.0,
    ) -> float:
        """Returns regen demand factor [0, 1]."""
        if brake <= 0:
            return 0.0
        fade = float(np.clip(vehicle_speed / max(fade_speed, 0.1), 0.0, 1.0))
        return float(np.clip(brake * regen_level * fade, 0.0, 1.0))
