"""Assembly mass properties."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable
import numpy as np
from .component import Component


@dataclass
class MassProperties:
    total_mass: float
    cg: np.ndarray
    axle_load_front: float
    axle_load_rear: float
    breakdown: dict[str, float] = field(default_factory=dict)
    Izz_approx: float = 0.0  # polar yaw inertia approx about CG

    def to_dict(self) -> dict:
        return {
            "total_mass": self.total_mass,
            "cg": self.cg.tolist(),
            "axle_load_front": self.axle_load_front,
            "axle_load_rear": self.axle_load_rear,
            "breakdown": self.breakdown,
            "Izz_approx": self.Izz_approx,
        }


def compute_mass_properties(
    components: Iterable[Component],
    wheelbase: float = 2.70,
    front_axle_x: float = 0.0,
) -> MassProperties:
    comps = [c for c in components if c.mass > 0]
    if not comps:
        return MassProperties(0.0, np.zeros(3), 0.0, 0.0)
    m = np.array([c.mass for c in comps])
    cg_pts = np.array([c.cg_global for c in comps])
    total = float(np.sum(m))
    cg = (m[:, None] * cg_pts).sum(axis=0) / total
    rear_axle_x = front_axle_x - wheelbase
    # static axle loads from longitudinal CG
    # load_front * 0 + load_rear * wheelbase = total * (cg_x - front)  with load_f + load_r = total
    # load_r = total * (cg_x - front) / (rear - front) but rear-front = -wb
    dx = cg[0] - front_axle_x
    # fraction to rear = dx / wheelbase if front at 0 and rear at -wb... use abs wheelbase
    # front load higher when CG closer to front
    # distance front-to-cg along x (front at front_axle_x, rear at front_axle_x - wb)
    # rear_load = total * (cg_x - front_x) / (rear_x - front_x) wait:
    # standard: F_f = m*g * (L_r / L), L_r = rear_x distance from CG
    L = wheelbase
    L_r = (front_axle_x - wheelbase) - cg[0]  # negative if typical
    # better: position of CG from rear axle
    dist_from_rear = cg[0] - (front_axle_x - wheelbase)
    dist_from_front = (front_axle_x) - cg[0]
    # if front at 0, rear at -L, cg at -a (a from front), then front_load = m*(L-a)/L = m * |rear-cg|/L
    rear_axle = front_axle_x - wheelbase
    load_front = total * (cg[0] - rear_axle) / L if abs(L) > 1e-9 else total * 0.5
    load_rear = total - load_front
    breakdown = {}
    for c in comps:
        breakdown[c.category] = breakdown.get(c.category, 0.0) + c.mass
    # yaw inertia approx: sum m * r_xy^2
    r2 = (cg_pts[:, 0] - cg[0]) ** 2 + (cg_pts[:, 1] - cg[1]) ** 2
    Izz = float(np.sum(m * r2))
    return MassProperties(total, cg, float(load_front), float(load_rear), breakdown, Izz)
