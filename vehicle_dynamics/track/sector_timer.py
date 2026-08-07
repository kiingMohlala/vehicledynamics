"""Sector timing, best sectors, delta, ghost comparison."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class SectorTimes:
    splits: list[float]          # cumulative time at sector ends
    sector_dt: list[float]       # time per sector
    total: float

    @property
    def n_sectors(self) -> int:
        return len(self.sector_dt)


@dataclass
class SectorTimer:
    boundaries_s: list[float]    # sector end distances (include finish)
    _last_idx: int = 0
    _times: list[float] = field(default_factory=list)
    _crossed: list[float] = field(default_factory=list)

    def reset(self) -> None:
        self._last_idx = 0
        self._times = []
        self._crossed = []

    def update(self, s: float, t: float) -> list[float]:
        """Return list of newly crossed sector times (may be empty)."""
        new = []
        while self._last_idx < len(self.boundaries_s) and s >= self.boundaries_s[self._last_idx]:
            self._crossed.append(t)
            new.append(t)
            self._last_idx += 1
        return new

    def result(self) -> SectorTimes | None:
        if len(self._crossed) < len(self.boundaries_s):
            return None
        splits = list(self._crossed)
        dts = [splits[0]] + [splits[i] - splits[i - 1] for i in range(1, len(splits))]
        return SectorTimes(splits=splits, sector_dt=dts, total=splits[-1])


def equal_sectors(track_length: float, n: int = 3) -> list[float]:
    n = max(1, n)
    return [track_length * (i + 1) / n for i in range(n)]


def best_sectors(laps: list[SectorTimes]) -> list[float]:
    if not laps:
        return []
    n = min(len(L.sector_dt) for L in laps)
    return [min(L.sector_dt[i] for L in laps) for i in range(n)]


def delta_to_ghost(current: SectorTimes, ghost: SectorTimes) -> list[float]:
    n = min(len(current.sector_dt), len(ghost.sector_dt))
    return [current.sector_dt[i] - ghost.sector_dt[i] for i in range(n)]
