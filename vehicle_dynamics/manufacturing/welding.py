"""Weld joint estimation."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WeldEstimate:
    length_m: float
    process: str  # MIG, TIG, laser
    time_hours: float
    cost: float
    mass_kg: float


def estimate_weld(
    length_m: float,
    process: str = "MIG",
    deposition_kg_per_m: float = 0.05,
    rate_m_per_hour: float = 12.0,
    hourly_cost: float = 65.0,
    filler_cost_per_kg: float = 8.0,
) -> WeldEstimate:
    rates = {"MIG": 15.0, "TIG": 6.0, "laser": 25.0}
    rate = rates.get(process, rate_m_per_hour)
    hours = length_m / max(rate, 0.1) + 0.25  # setup
    mass = length_m * deposition_kg_per_m
    cost = hours * hourly_cost + mass * filler_cost_per_kg
    return WeldEstimate(length_m, process, hours, cost, mass)
