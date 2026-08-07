"""CNC time estimation."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class MachiningEstimate:
    volume_removed_cm3: float
    time_hours: float
    cost: float
    features: int


def estimate_cnc(
    stock_volume_cm3: float,
    final_volume_cm3: float,
    features: int = 10,
    mr_rate_cm3_per_min: float = 5.0,
    hourly_cost: float = 85.0,
    setup_hours: float = 0.5,
) -> MachiningEstimate:
    removed = max(stock_volume_cm3 - final_volume_cm3, 0.0)
    machine_min = removed / max(mr_rate_cm3_per_min, 0.1) + features * 1.5
    hours = setup_hours + machine_min / 60.0
    return MachiningEstimate(removed, hours, hours * hourly_cost, features)
