"""Simple cycle-counting capacity fade / resistance growth."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class BatteryDegradation:
    capacity_fade: float = 0.0          # fraction lost
    r_growth: float = 0.0               # fraction R increase
    cycle_count: float = 0.0
    throughput_kwh: float = 0.0
    fade_per_full_cycle: float = 0.00015
    r_growth_per_cycle: float = 0.0001

    def update(self, energy_throughput_kwh: float, dod: float = 0.5) -> None:
        """Accumulate equivalent full cycles from energy throughput."""
        self.throughput_kwh += abs(energy_throughput_kwh)
        # Approximate: 1 full cycle ≈ 2 * capacity at given DoD
        eq = abs(energy_throughput_kwh) / max(dod, 0.1)
        self.cycle_count += eq
        self.capacity_fade = float(min(0.3, self.cycle_count * self.fade_per_full_cycle))
        self.r_growth = float(min(0.5, self.cycle_count * self.r_growth_per_cycle))

    @property
    def capacity_factor(self) -> float:
        return float(np.clip(1.0 - self.capacity_fade, 0.5, 1.0))

    @property
    def r_factor(self) -> float:
        return float(1.0 + self.r_growth)
