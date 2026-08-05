"""Simple coolant / oil thermal model."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ThermalState:
    coolant_C: float = 20.0
    oil_C: float = 20.0
    efficiency_factor: float = 0.85  # cold penalty


@dataclass
class ThermalModel:
    ambient_C: float = 20.0
    target_coolant_C: float = 90.0
    target_oil_C: float = 100.0
    tau_coolant: float = 40.0   # s
    tau_oil: float = 60.0
    heat_gain_kw_per_kw: float = 1.2

    def step(
        self,
        power_kw: float,
        dt: float,
        state: ThermalState,
    ) -> ThermalState:
        # Heat input from combustion proxy
        q = max(power_kw, 0.5) * self.heat_gain_kw_per_kw
        # First-order approach to load-dependent temps
        cool_tgt = self.ambient_C + (self.target_coolant_C - self.ambient_C) * np.clip(q / 80.0, 0.2, 1.2)
        oil_tgt = self.ambient_C + (self.target_oil_C - self.ambient_C) * np.clip(q / 70.0, 0.2, 1.3)
        a_c = 1.0 - np.exp(-dt / self.tau_coolant) if dt > 0 else 1.0
        a_o = 1.0 - np.exp(-dt / self.tau_oil) if dt > 0 else 1.0
        cool = state.coolant_C + a_c * (cool_tgt - state.coolant_C)
        oil = state.oil_C + a_o * (oil_tgt - state.oil_C)
        # Efficiency: cold engines richer / more friction
        warm = np.clip((cool - 40.0) / 50.0, 0.0, 1.0)
        eff = 0.80 + 0.20 * warm
        return ThermalState(coolant_C=float(cool), oil_C=float(oil), efficiency_factor=float(eff))
