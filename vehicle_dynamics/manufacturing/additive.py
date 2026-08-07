"""Additive manufacturing estimates."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AMEstimate:
    volume_cm3: float
    time_hours: float
    cost: float
    process: str


def estimate_am(
    volume_cm3: float,
    process: str = "FDM",
    rate_cm3_per_hour: float = 30.0,
    material_cost_per_cm3: float = 0.15,
    machine_hourly: float = 15.0,
) -> AMEstimate:
    rates = {"FDM": 40.0, "SLA": 25.0, "SLS": 8.0, "DMLS": 5.0}
    rate = rates.get(process, rate_cm3_per_hour)
    hours = volume_cm3 / max(rate, 0.1) + 0.5
    cost = hours * machine_hourly + volume_cm3 * material_cost_per_cm3
    return AMEstimate(volume_cm3, hours, cost, process)
