"""Battery thermal model + derating."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class BatteryThermal:
    temp_c: float = 25.0
    mass_kg: float = 120.0
    cp: float = 800.0           # J/(kg·K)
    cooling_w_per_k: float = 40.0
    ambient_c: float = 25.0
    t_opt: float = 25.0
    t_max: float = 55.0
    t_min: float = -10.0

    def step(self, power_loss_w: float, dt: float) -> float:
        """Update temperature; return derate factor in [0, 1]."""
        cool = self.cooling_w_per_k * (self.temp_c - self.ambient_c)
        dT = (power_loss_w - cool) * dt / max(self.mass_kg * self.cp, 1.0)
        self.temp_c = float(np.clip(self.temp_c + dT, -40.0, 80.0))
        return self.derate()

    def derate(self) -> float:
        t = self.temp_c
        if self.t_min <= t <= self.t_max:
            # mild derate outside ±15°C of optimum
            return float(np.clip(1.0 - 0.01 * abs(t - self.t_opt), 0.3, 1.0))
        if t > self.t_max:
            return float(np.clip(1.0 - 0.05 * (t - self.t_max), 0.0, 0.5))
        return float(np.clip(1.0 - 0.03 * (self.t_min - t), 0.0, 0.5))
