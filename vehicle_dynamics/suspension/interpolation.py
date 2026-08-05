"""1-D interpolation of geometry curves vs wheel travel."""

from __future__ import annotations

import numpy as np
from .geometry_curves import GeometryCurves


def interp_scalar(travel: np.ndarray, values: np.ndarray, z: float) -> float:
    z = float(np.clip(z, travel[0], travel[-1]))
    return float(np.interp(z, travel, values))


class NonlinearGeometryLookup:
    """Query camber/toe/RC/... at arbitrary wheel travel."""

    def __init__(self, curves: GeometryCurves):
        self.curves = curves

    def camber_rad(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.camber_rad, z)

    def toe_rad(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.toe_rad, z)

    def roll_center_z(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.roll_center_z, z)

    def kpi_rad(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.kpi_rad, z)

    def caster_rad(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.caster_rad, z)

    def scrub_radius(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.scrub_radius, z)

    def trail(self, z: float) -> float:
        return interp_scalar(self.curves.travel, self.curves.trail, z)

    def evaluate(self, z: float) -> dict:
        return {
            "camber_rad": self.camber_rad(z),
            "toe_rad": self.toe_rad(z),
            "kpi_rad": self.kpi_rad(z),
            "caster_rad": self.caster_rad(z),
            "roll_center_z": self.roll_center_z(z),
            "scrub_radius": self.scrub_radius(z),
            "trail": self.trail(z),
            "wheel_travel": float(z),
        }
