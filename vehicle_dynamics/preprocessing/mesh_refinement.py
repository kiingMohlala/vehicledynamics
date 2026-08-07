"""Local refinement zone definitions and edge-length maps."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable
import numpy as np


@dataclass
class RefinementZone:
    name: str
    center: np.ndarray
    radius: float
    target_size: float

    def contains(self, point: np.ndarray) -> bool:
        return float(np.linalg.norm(point - self.center)) <= self.radius


@dataclass
class RefinementMap:
    zones: list[RefinementZone] = field(default_factory=list)
    default_size: float = 0.05

    def size_at(self, point: np.ndarray) -> float:
        sizes = [self.default_size]
        for z in self.zones:
            if z.contains(point):
                sizes.append(z.target_size)
        return float(min(sizes))

    def add_leading_edge(self, center, radius=0.15, size=0.005) -> None:
        self.zones.append(RefinementZone("leading_edge", np.asarray(center, dtype=float), radius, size))

    def add_wheel(self, center, radius=0.40, size=0.008) -> None:
        self.zones.append(RefinementZone("wheel", np.asarray(center, dtype=float), radius, size))

    def add_diffuser(self, center, radius=0.50, size=0.01) -> None:
        self.zones.append(RefinementZone("diffuser", np.asarray(center, dtype=float), radius, size))
