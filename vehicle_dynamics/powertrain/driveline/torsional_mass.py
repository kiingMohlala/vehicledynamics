"""Lumped rotational inertias along the driveline."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TorsionalInertia:
    """J θ_ddot = Σ T."""

    J: float = 0.15  # kg·m²
    name: str = "inertia"

    def accel(self, torque_net: float) -> float:
        if self.J <= 1e-12:
            return 0.0
        return float(torque_net / self.J)
