"""Static solution result container."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class StaticResult:
    u: np.ndarray              # full displacement vector (ndof,)
    reactions: np.ndarray      # reaction forces at constrained DOFs (ndof,)
    success: bool
    message: str = ""

    def node_displacement(self, node_id: int) -> np.ndarray:
        base = 6 * node_id
        return self.u[base : base + 6].copy()
