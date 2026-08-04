"""FEM nodes (6 DOF)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Node:
    id: int
    x: float
    y: float
    z: float
    fixed: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=bool))
    tag: str = ""

    def coords(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def dof_indices(self) -> np.ndarray:
        base = 6 * self.id
        return np.arange(base, base + 6)
