"""Body panel representation built from parametric surfaces."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class Panel:
    name: str
    kind: str  # hood, roof, door, fender, floor, undertray, splitter, diffuser, deck, wing
    surface: Any  # object with evaluate(u,v) and sample_grid(nu,nv)
    u_range: tuple[float, float] = (0.0, 1.0)
    v_range: tuple[float, float] = (0.0, 1.0)
    meta: dict[str, Any] = field(default_factory=dict)

    def sample_grid(self, nu: int = 20, nv: int = 20) -> np.ndarray:
        return self.surface.sample_grid(nu, nv)

    def evaluate(self, u: float, v: float) -> np.ndarray:
        return self.surface.evaluate(u, v)
