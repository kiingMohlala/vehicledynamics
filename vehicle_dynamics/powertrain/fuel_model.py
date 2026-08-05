"""BSFC-based fuel consumption."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class FuelState:
    fuel_rate_gps: float = 0.0   # g/s
    fuel_total_g: float = 0.0
    bsfc_gpkwh: float = 250.0


@dataclass
class FuelModel:
    bsfc_idle: float = 350.0      # g/kWh
    bsfc_best: float = 230.0
    bsfc_high_load: float = 280.0
    density: float = 740.0        # g/L gasoline approx for reports

    def bsfc(self, throttle: float, rpm: float, redline: float) -> float:
        # Best BSFC mid load mid rpm
        load = float(np.clip(throttle, 0, 1))
        rpm_n = float(np.clip(rpm / max(redline, 1), 0, 1.2))
        mid = np.exp(-((load - 0.55) ** 2) / 0.15) * np.exp(-((rpm_n - 0.5) ** 2) / 0.2)
        bsfc = self.bsfc_high_load * (1 - mid) + self.bsfc_best * mid
        if load < 0.08:
            bsfc = self.bsfc_idle
        return float(bsfc)

    def step(
        self,
        power_kw: float,
        throttle: float,
        rpm: float,
        redline: float,
        dt: float,
        state: FuelState,
    ) -> FuelState:
        b = self.bsfc(throttle, rpm, redline)
        p = max(power_kw, 0.0)
        # Idle fuel floor
        rate = b * p / 3600.0  # g/s from g/kWh * kW
        if throttle < 0.05 and p < 1.0:
            rate = max(rate, 0.15)  # ~idle injector floor
        total = state.fuel_total_g + rate * dt
        return FuelState(fuel_rate_gps=rate, fuel_total_g=total, bsfc_gpkwh=b)
