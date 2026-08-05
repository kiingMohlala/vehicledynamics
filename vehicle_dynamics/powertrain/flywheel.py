"""Rotational inertia / flywheel energy."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Flywheel:
    inertia: float = 0.25  # kg·m² effective engine+flywheel

    def energy(self, omega: float) -> float:
        return 0.5 * self.inertia * omega * omega

    def alpha(self, torque_net: float) -> float:
        """Angular acceleration rad/s²."""
        return torque_net / max(self.inertia, 1e-6)
