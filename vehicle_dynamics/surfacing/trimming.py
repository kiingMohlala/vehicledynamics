"""Simple surface domain trimming and panel stitching metadata."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Any
import numpy as np


@dataclass
class TrimDomain:
    """Keep points where predicate(u,v) is True."""
    predicate: Callable[[float, float], bool]

    def keep(self, u: float, v: float) -> bool:
        return bool(self.predicate(u, v))


@dataclass
class TrimmedSurface:
    surface: Any
    domain: TrimDomain

    def evaluate(self, u: float, v: float) -> np.ndarray:
        if not self.domain.keep(u, v):
            return np.full(3, np.nan)
        return np.asarray(self.surface.evaluate(u, v), dtype=float)

    def sample_grid(self, nu: int = 20, nv: int = 20) -> np.ndarray:
        g = np.zeros((nu, nv, 3))
        for i, u in enumerate(np.linspace(0, 1, nu)):
            for j, v in enumerate(np.linspace(0, 1, nv)):
                g[i, j] = self.evaluate(u, v)
        return g


@dataclass
class StitchedBody:
    """Collection of panels treated as one body."""
    panels: list = field(default_factory=list)

    def add(self, panel) -> None:
        self.panels.append(panel)

    def __len__(self) -> int:
        return len(self.panels)
