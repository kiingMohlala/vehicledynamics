"""Design constraints (ride height, stall, packaging)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .design_variables import DesignVector, DesignBounds, default_bounds


@dataclass
class ConstraintSet:
    h_front_min: float = 0.035
    h_rear_min: float = 0.040
    max_rake: float = 0.08
    max_drag: float = 2500.0
    max_front_balance: float = 0.55
    min_front_balance: float = 0.25
    max_wing_angle: float = 0.28
    ground_clearance_min: float = 0.030


def evaluate_constraints(
    design: DesignVector,
    drag: float,
    front_balance: float,
    cs: ConstraintSet | None = None,
) -> tuple[bool, dict[str, float]]:
    """
    Returns (feasible, violations) where violations[name] > 0 means violated.
    """
    cs = cs or ConstraintSet()
    v: dict[str, float] = {}
    v["h_front"] = cs.h_front_min - design.h_front
    v["h_rear"] = cs.h_rear_min - design.h_rear
    v["rake"] = abs(design.rake) - cs.max_rake
    v["drag"] = drag - cs.max_drag
    v["bal_hi"] = front_balance - cs.max_front_balance
    v["bal_lo"] = cs.min_front_balance - front_balance
    v["rw_angle"] = design.rear_wing_angle - cs.max_wing_angle
    v["fw_angle"] = design.front_wing_angle - cs.max_wing_angle
    v["clearance"] = cs.ground_clearance_min - min(design.h_front, design.h_rear)
    feasible = all(val <= 1e-9 for val in v.values())
    return feasible, v
