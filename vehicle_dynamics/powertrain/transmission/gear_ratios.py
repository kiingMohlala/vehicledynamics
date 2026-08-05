"""Gear and final-drive ratios."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class GearRatios:
    """
    gear_ratios[0] unused; index 1..n = forward gears.
    reverse is negative ratio.
    """

    primary: float = 1.0
    final_drive: float = 3.90
    efficiency: float = 0.95
    gears: list[float] = field(
        default_factory=lambda: [0.0, 3.50, 2.20, 1.60, 1.20, 1.00, 0.85]
    )
    reverse: float = -3.20

    @property
    def n_forward(self) -> int:
        return max(0, len(self.gears) - 1)

    def ratio(self, gear: int) -> float:
        if gear == 0:
            return 0.0
        if gear < 0:
            return self.reverse * self.primary
        if gear >= len(self.gears):
            gear = len(self.gears) - 1
        return self.gears[gear] * self.primary

    def overall(self, gear: int) -> float:
        return self.ratio(gear) * self.final_drive

    def output_torque(self, t_in: float, gear: int) -> float:
        return t_in * self.overall(gear) * self.efficiency

    def output_omega(self, omega_in: float, gear: int) -> float:
        r = self.overall(gear)
        if abs(r) < 1e-9:
            return 0.0
        return omega_in / r


def default_ratios(final_drive: float = 3.90) -> GearRatios:
    return GearRatios(final_drive=final_drive)
