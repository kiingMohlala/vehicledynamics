"""Open-loop command profiles."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class OpenLoopProfile:
    """Time-indexed throttle/brake/steer tables."""

    t: np.ndarray = field(default_factory=lambda: np.array([0.0, 1.0]))
    throttle: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    brake: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    steer: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))

    def at(self, time: float) -> tuple[float, float, float]:
        thr = float(np.interp(time, self.t, self.throttle))
        br = float(np.interp(time, self.t, self.brake))
        st = float(np.interp(time, self.t, self.steer))
        return thr, br, st
