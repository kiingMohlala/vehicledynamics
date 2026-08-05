"""Differential base result container."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DiffResult:
    torque_left: float = 0.0
    torque_right: float = 0.0
    bias: float = 0.0
    locking_fraction: float = 0.0
    axle_speed: float = 0.0
    delta_omega: float = 0.0


class DifferentialBase:
    """Marker base for type-specific splitters."""

    name: str = "base"
