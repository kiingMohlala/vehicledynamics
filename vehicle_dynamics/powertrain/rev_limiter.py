"""Rev limiter (hard/soft cut)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np


class LimitMode(str, Enum):
    HARD = "hard"
    SOFT = "soft"


@dataclass
class RevLimiter:
    redline_rpm: float = 7500.0
    soft_start_rpm: float = 7300.0
    mode: LimitMode = LimitMode.SOFT

    def factor(self, rpm: float) -> float:
        """Torque scale 0..1."""
        if rpm < self.soft_start_rpm:
            return 1.0
        if self.mode == LimitMode.HARD:
            return 0.0 if rpm >= self.redline_rpm else 1.0
        # Soft: linear fade soft_start → redline
        if rpm >= self.redline_rpm:
            return 0.0
        span = max(self.redline_rpm - self.soft_start_rpm, 1.0)
        return float(np.clip(1.0 - (rpm - self.soft_start_rpm) / span, 0.0, 1.0))
