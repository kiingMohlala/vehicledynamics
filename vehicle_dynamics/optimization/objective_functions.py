"""Objective functions for design evaluation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Any


@dataclass
class Objective:
    name: str
    fn: Callable[[dict[str, Any]], float]
    sense: str = "minimize"  # minimize | maximize
    weight: float = 1.0

    def evaluate(self, result: dict[str, Any]) -> float:
        val = float(self.fn(result))
        return val if self.sense == "minimize" else -val


def lap_time_objective() -> Objective:
    return Objective("lap_time", lambda r: float(r.get("lap_time", 1e6)), "minimize")


def energy_objective() -> Objective:
    return Objective("energy", lambda r: float(r.get("energy", r.get("fuel_used", 0.0))), "minimize")


def top_speed_objective() -> Objective:
    return Objective("top_speed", lambda r: float(r.get("top_speed", 0.0)), "maximize")


def comfort_objective() -> Objective:
    return Objective("comfort_rms_ax", lambda r: float(r.get("rms_ax", r.get("max_ax", 0.0))), "minimize")


def composite(objectives: list[Objective], result: dict[str, Any]) -> float:
    return sum(o.weight * o.evaluate(result) for o in objectives)
