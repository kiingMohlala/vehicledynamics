"""
Sample suspension metrics vs wheel travel for interpolation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .hardpoints import WishboneHardpoints, default_front_left
from .travel_solver import solve_at_travel


@dataclass
class GeometryCurves:
    """Tabulated nonlinear geometry vs wheel travel [m]."""
    travel: np.ndarray  # shape (N,)
    camber_rad: np.ndarray
    toe_rad: np.ndarray
    kpi_rad: np.ndarray
    caster_rad: np.ndarray
    roll_center_z: np.ndarray
    scrub_radius: np.ndarray
    trail: np.ndarray
    ic_y: np.ndarray
    ic_z: np.ndarray

    def at_zero(self) -> dict:
        i = int(np.argmin(np.abs(self.travel)))
        return {
            "camber_rad": float(self.camber_rad[i]),
            "toe_rad": float(self.toe_rad[i]),
            "kpi_rad": float(self.kpi_rad[i]),
            "caster_rad": float(self.caster_rad[i]),
            "roll_center_z": float(self.roll_center_z[i]),
        }


def build_curves(
    hp: WishboneHardpoints = None,
    travel_min: float = -0.08,
    travel_max: float = 0.08,
    n: int = 41,
) -> GeometryCurves:
    """
    Dense sample of geometry over travel range.
    NaN samples (e.g. parallel arms) are filled by nearest finite neighbour.
    """
    hp = hp or default_front_left()
    travel = np.linspace(travel_min, travel_max, n)
    camber = np.full(n, np.nan)
    toe = np.full(n, np.nan)
    kpi = np.full(n, np.nan)
    caster = np.full(n, np.nan)
    rc = np.full(n, np.nan)
    scrub = np.full(n, np.nan)
    trail = np.full(n, np.nan)
    ic_y = np.full(n, np.nan)
    ic_z = np.full(n, np.nan)

    for i, z in enumerate(travel):
        r = solve_at_travel(hp, float(z))
        camber[i] = np.radians(r.camber_deg)
        toe[i] = np.radians(r.toe_deg)
        kpi[i] = np.radians(r.kpi_deg)
        caster[i] = np.radians(r.caster_deg)
        rc[i] = r.roll_center_z
        scrub[i] = r.scrub_radius
        trail[i] = r.trail
        ic_y[i] = r.instant_center_y
        ic_z[i] = r.instant_center_z

    def fill_nan(a: np.ndarray) -> np.ndarray:
        out = a.copy()
        finite = np.isfinite(out)
        if not np.any(finite):
            return np.zeros_like(out)
        idx = np.arange(len(out))
        out[~finite] = np.interp(idx[~finite], idx[finite], out[finite])
        return out

    return GeometryCurves(
        travel=travel,
        camber_rad=fill_nan(camber),
        toe_rad=fill_nan(toe),
        kpi_rad=fill_nan(kpi),
        caster_rad=fill_nan(caster),
        roll_center_z=fill_nan(rc),
        scrub_radius=fill_nan(scrub),
        trail=fill_nan(trail),
        ic_y=fill_nan(ic_y),
        ic_z=fill_nan(ic_z),
    )
