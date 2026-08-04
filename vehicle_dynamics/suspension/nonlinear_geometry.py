"""
Phase 6.6 – Nonlinear geometry solver facade.

camber = solver.solve(z).camber   (not gain × z)
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .hardpoints import WishboneHardpoints, default_front_left, mirror_corner
from .geometry_curves import build_curves, GeometryCurves
from .interpolation import NonlinearGeometryLookup
from .travel_solver import solve_at_travel, solve_static
from .wishbone import analyze


@dataclass
class NonlinearCornerState:
    wheel_travel: float
    camber_rad: float
    toe_rad: float
    kpi_rad: float
    caster_rad: float
    roll_center_z: float
    scrub_radius: float
    trail: float


class NonlinearGeometrySolver:
    """
    Precomputes geometry curves for a corner and provides fast lookup.

    solve(0) matches Phase 6.0/6.5 static analyze() within tolerance.
    """

    def __init__(
        self,
        hp: WishboneHardpoints = None,
        travel_min: float = -0.08,
        travel_max: float = 0.08,
        n: int = 41,
    ):
        self.hp = hp or default_front_left()
        self.curves = build_curves(self.hp, travel_min, travel_max, n)
        self.lookup = NonlinearGeometryLookup(self.curves)
        self._static = solve_static(self.hp)

    def solve(self, wheel_travel: float) -> NonlinearCornerState:
        d = self.lookup.evaluate(float(wheel_travel))
        return NonlinearCornerState(
            wheel_travel=d["wheel_travel"],
            camber_rad=d["camber_rad"],
            toe_rad=d["toe_rad"],
            kpi_rad=d["kpi_rad"],
            caster_rad=d["caster_rad"],
            roll_center_z=d["roll_center_z"],
            scrub_radius=d["scrub_radius"],
            trail=d["trail"],
        )

    def solve_exact(self, wheel_travel: float) -> NonlinearCornerState:
        """Direct hardpoint solve (no interpolation)."""
        r = solve_at_travel(self.hp, float(wheel_travel))
        return NonlinearCornerState(
            wheel_travel=float(wheel_travel),
            camber_rad=np.radians(r.camber_deg),
            toe_rad=np.radians(r.toe_deg),
            kpi_rad=np.radians(r.kpi_deg),
            caster_rad=np.radians(r.caster_deg),
            roll_center_z=r.roll_center_z,
            scrub_radius=r.scrub_radius,
            trail=r.trail,
        )


class FourCornerNonlinearGeometry:
    """Independent nonlinear solvers for FL/FR/RL/RR."""

    def __init__(self, front_left: WishboneHardpoints = None):
        fl = front_left or default_front_left()
        fr = mirror_corner(fl)
        self.fl = NonlinearGeometrySolver(fl)
        self.fr = NonlinearGeometrySolver(fr)
        self.rl = NonlinearGeometrySolver(fl)  # illustrative rear copy
        self.rr = NonlinearGeometrySolver(fr)

    def evaluate(self, wheel_travel: np.ndarray) -> list[NonlinearCornerState]:
        z = np.asarray(wheel_travel, dtype=float).reshape(4)
        return [
            self.fl.solve(z[0]),
            self.fr.solve(z[1]),
            self.rl.solve(z[2]),
            self.rr.solve(z[3]),
        ]
