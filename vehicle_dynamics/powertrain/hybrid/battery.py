"""Lumped battery pack: SOC, OCV, resistance, power limits."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class BatteryConfig:
    capacity_kwh: float = 18.0
    capacity_ah: float = 50.0       # derived if needed
    v_nom: float = 360.0
    v_min: float = 280.0
    v_max: float = 400.0
    r_internal: float = 0.08        # ohm
    i_max_charge: float = 200.0     # A
    i_max_discharge: float = 400.0
    soc0: float = 0.70
    soc_min: float = 0.05
    soc_max: float = 0.95


@dataclass
class BatteryState:
    soc: float = 0.70
    voltage: float = 360.0
    current: float = 0.0            # + discharge, - charge
    power_kw: float = 0.0           # + out to inverter
    available_discharge_kw: float = 0.0
    available_charge_kw: float = 0.0
    energy_used_kwh: float = 0.0
    energy_recovered_kwh: float = 0.0


class Battery:
    def __init__(self, config: BatteryConfig | None = None):
        self.cfg = config or BatteryConfig()
        # Prefer capacity_kwh; Ah ≈ kWh*1000/Vnom
        if self.cfg.capacity_ah <= 0:
            self.cfg.capacity_ah = self.cfg.capacity_kwh * 1000.0 / max(self.cfg.v_nom, 1.0)
        self.soc = float(np.clip(self.cfg.soc0, self.cfg.soc_min, self.cfg.soc_max))
        self.energy_used = 0.0
        self.energy_rec = 0.0
        self.state = BatteryState(soc=self.soc, voltage=self._ocv(self.soc))

    def _ocv(self, soc: float) -> float:
        """Simple linear OCV vs SOC."""
        c = self.cfg
        return c.v_min + (c.v_max - c.v_min) * float(np.clip(soc, 0, 1))

    def available_power(self, temp_derate: float = 1.0) -> tuple[float, float]:
        """(discharge_kw, charge_kw) limits."""
        v = self._ocv(self.soc)
        d = min(self.cfg.i_max_discharge, v / max(self.cfg.r_internal, 1e-6)) * v / 1000.0
        ch = min(self.cfg.i_max_charge, v / max(self.cfg.r_internal, 1e-6)) * v / 1000.0
        d *= temp_derate
        ch *= temp_derate
        if self.soc <= self.cfg.soc_min:
            d = 0.0
        if self.soc >= self.cfg.soc_max:
            ch = 0.0
        return max(d, 0.0), max(ch, 0.0)

    def step(self, power_kw: float, dt: float, temp_derate: float = 1.0) -> BatteryState:
        """
        power_kw > 0: discharge (to motor)
        power_kw < 0: charge (from regen / charger)
        """
        c = self.cfg
        p_d_max, p_c_max = self.available_power(temp_derate)
        p = float(power_kw)
        if p > 0:
            p = min(p, p_d_max)
        else:
            p = max(p, -p_c_max)

        v_ocv = self._ocv(self.soc)
        # I ≈ P / V  (first-order; ignore I²R in SOC for simplicity)
        if abs(v_ocv) < 1.0:
            i = 0.0
        else:
            i = (p * 1000.0) / v_ocv
        i = float(np.clip(i, -c.i_max_charge, c.i_max_discharge))
        v_term = v_ocv - i * c.r_internal

        # SOC update: ΔSOC = -I dt / (Ah * 3600)
        q = c.capacity_ah * 3600.0
        self.soc = float(np.clip(self.soc - i * dt / max(q, 1e-6), c.soc_min, c.soc_max))

        e = p * dt / 3600.0  # kWh
        if p > 0:
            self.energy_used += e
        else:
            self.energy_rec += -e

        self.state = BatteryState(
            soc=self.soc,
            voltage=float(v_term),
            current=i,
            power_kw=p,
            available_discharge_kw=p_d_max,
            available_charge_kw=p_c_max,
            energy_used_kwh=self.energy_used,
            energy_recovered_kwh=self.energy_rec,
        )
        return self.state

    def reset(self, soc: float | None = None) -> None:
        self.soc = float(np.clip(soc if soc is not None else self.cfg.soc0, self.cfg.soc_min, self.cfg.soc_max))
        self.energy_used = 0.0
        self.energy_rec = 0.0
        self.state = BatteryState(soc=self.soc, voltage=self._ocv(self.soc))
