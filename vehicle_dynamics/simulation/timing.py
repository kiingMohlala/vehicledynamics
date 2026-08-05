"""Fixed timestep management."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FixedTimestep:
    dt: float = 0.01

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if self.dt > 0.1:
            raise ValueError("dt too large for stable integration; use <= 0.1 s")

    def n_steps(self, duration: float) -> int:
        return int(max(1, round(duration / self.dt)))
