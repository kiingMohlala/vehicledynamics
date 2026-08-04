"""
Phase 6.5 – Dynamic roll-center migration.

Displaces outer hardpoints with wheel travel, then recomputes instant center
and roll-center height using the Phase 6.0 geometric construction.

When arms become parallel (IC at infinity), falls back to the design-position
RC for that corner so results remain finite.

Diagnostic only: no jacking forces, no load-transfer feedback.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .hardpoints import (
    WishboneHardpoints,
    Point3,
    default_front_left,
    mirror_corner,
)
from .geometry import average_inner, instant_center_yz
from .wishbone import roll_center_front_view
from .roll_center_state import RollCenterState


def displace_corner(hp: WishboneHardpoints, dz: float) -> WishboneHardpoints:
    """
    Approximate wheel bump by vertically shifting outer upright attachments
    and wheel center. Contact patch stays on ground (z=0). Inner pivots fixed.
    """
    dz = float(dz)

    def up(p: Point3) -> Point3:
        return Point3(p.x, p.y, p.z + dz)

    return WishboneHardpoints(
        upper_front=hp.upper_front,
        upper_rear=hp.upper_rear,
        upper_outer=up(hp.upper_outer),
        lower_front=hp.lower_front,
        lower_rear=hp.lower_rear,
        lower_outer=up(hp.lower_outer),
        tierod_inner=hp.tierod_inner,
        tierod_outer=up(hp.tierod_outer),
        wheel_center=up(hp.wheel_center),
        contact_patch=Point3(hp.contact_patch.x, hp.contact_patch.y, 0.0),
    )


def corner_ic_yz(hp: WishboneHardpoints) -> tuple[float, float] | None:
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    return instant_center_yz(
        ui.y, ui.z,
        hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z,
        hp.lower_outer.y, hp.lower_outer.z,
    )


def corner_roll_center_z(hp: WishboneHardpoints) -> float:
    rc = roll_center_front_view(hp)
    if rc is None:
        return float("nan")
    return float(rc[1])


@dataclass
class RollCenterGeometry:
    """Hardpoints for four corners (design position)."""
    fl: WishboneHardpoints = None
    fr: WishboneHardpoints = None
    rl: WishboneHardpoints = None
    rr: WishboneHardpoints = None

    def __post_init__(self):
        if self.fl is None:
            self.fl = default_front_left()
        if self.fr is None:
            self.fr = mirror_corner(self.fl)
        if self.rl is None:
            self.rl = default_front_left()
        if self.rr is None:
            self.rr = mirror_corner(self.rl)


def compute_roll_centers(
    wheel_travel: np.ndarray,
    geometry: RollCenterGeometry = None,
) -> RollCenterState:
    """
    Parameters
    ----------
    wheel_travel : (4,) [m] FL, FR, RL, RR  (+ = compression / wheel up)

    Front RC = average of left/right corner RC heights after displacement.
    Parallel-arm corners fall back to design RC (finite).
    """
    geometry = geometry or RollCenterGeometry()
    z = np.asarray(wheel_travel, dtype=float).reshape(4)

    static_hp = [geometry.fl, geometry.fr, geometry.rl, geometry.rr]
    rc_static = np.array([corner_roll_center_z(h) for h in static_hp])

    corners_hp = [
        displace_corner(geometry.fl, z[0]),
        displace_corner(geometry.fr, z[1]),
        displace_corner(geometry.rl, z[2]),
        displace_corner(geometry.rr, z[3]),
    ]

    rc = np.zeros(4)
    ic_y = np.full(4, np.nan)
    ic_z = np.full(4, np.nan)
    for i, h in enumerate(corners_hp):
        val = corner_roll_center_z(h)
        if not np.isfinite(val):
            # parallel arms / IC at infinity → keep design RC
            val = rc_static[i]
        rc[i] = val
        ic = corner_ic_yz(h)
        if ic is not None:
            ic_y[i], ic_z[i] = ic

    rc_f = 0.5 * (rc[0] + rc[1])
    rc_r = 0.5 * (rc[2] + rc[3])
    rc_f0 = 0.5 * (rc_static[0] + rc_static[1])
    rc_r0 = 0.5 * (rc_static[2] + rc_static[3])

    return RollCenterState(
        wheel_travel=z.copy(),
        rc_front=float(rc_f),
        rc_rear=float(rc_r),
        rc_front_static=float(rc_f0),
        rc_rear_static=float(rc_r0),
        rc_front_migration=float(rc_f - rc_f0),
        rc_rear_migration=float(rc_r - rc_r0),
        ic_y=ic_y,
        ic_z=ic_z,
    )


class RollCenterModel:
    """Stateful wrapper for SuspensionInterface."""

    def __init__(self, geometry: RollCenterGeometry = None):
        self.geometry = geometry or RollCenterGeometry()
        self.last = compute_roll_centers(np.zeros(4), self.geometry)

    def reset(self):
        self.last = compute_roll_centers(np.zeros(4), self.geometry)

    def evaluate(self, wheel_travel: np.ndarray) -> RollCenterState:
        self.last = compute_roll_centers(wheel_travel, self.geometry)
        return self.last
