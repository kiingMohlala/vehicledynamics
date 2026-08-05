"""Static solution result container."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class StaticResult:
    u: np.ndarray
    reactions: np.ndarray
    success: bool
    message: str = ""
    # Optional engineering extras (Phase 8.1)
    element_stresses: dict[int, float] = field(default_factory=dict)
    max_displacement: float = 0.0
    torsional_stiffness_Nm_per_deg: float | None = None

    def node_displacement(self, node_id: int) -> np.ndarray:
        base = 6 * node_id
        return self.u[base : base + 6].copy()
