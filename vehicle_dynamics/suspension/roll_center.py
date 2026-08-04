"""
Phase 6.5 – Dynamic roll-center migration.

Recomputes front/rear roll-center height from hardpoints after applying
vertical wheel travel to the upright (outer ball joints).

  RC(t) = GeometrySolver(current wheel positions)

Diagnostic only — no jacking forces, no load-transfer feedback.

At z_FL = z_FR = z_RL = z_RR = 0 the result matches Phase 6.0.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .hardpoints import WishboneHardpoints, Point3, default_front_left, mirror_corner
from .geometry import instant_center_yz, average_inner
from .wishbone import roll_center_front_view, analyze
from .roll_center_state import RollCenterState


def _shift_outer_vertical(hp: WishboneHardpoints, z: float) -> WishboneHardpoints:
    """
    First-order upright motion: outer ball joints and wheel center move
    vertically with wheel travel; body pivots fixed; contact patch stays on ground.
    """
    z = float(z)

    def pz(p: Point3, dz: float) -> Point3:
        return Point3(p.x, p.y, p.z + dz)

    return WishboneHardpoints(
        upper_front=hp.upper_front,
        upper_rear=hp.upper_rear,
        upper_outer=pz(hp.upper_outer, z),
        lower_front=hp.lower_front,
        lower_rear=hp.lower_rear,
        lower_outer=pz(hp.lower_outer, z),
        tierod_inner=hp.tierod_inner,
        tierod_outer=pz(hp.tierod_outer, z),
        wheel_center=pz(hp.wheel_center, z),
        contact_patch=Point3(hp.contact_patch.x, hp.contact_patch.y, 0.0),
    )


def roll_center_height(hp: WishboneHardpoints) -> tuple[float, float, float]:
    """
    Returns (rc_z, ic_y, ic_z).
    rc_z may be nan if arms are parallel.
    """
    rc = roll_center_front_view(hp)
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    ic = instant_center_yz(
        ui.y, ui.z,
        hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z,
        hp.lower_outer.y, hp.lower_outer.z,
    )
    if rc is None:
        rc_z = float("nan")
    else:
        rc_z = float(rc[1])
    if ic is None:
        return rc_z, float("nan"), float("nan")
    return rc_z, float(ic[0]), float(ic[1])


@dataclass
class AxleHardpoints:
    """Left and right corner hardpoints for one axle."""
    left: WishboneHardpoints
    right: WishboneHardpoints


def default_front_axle() -> AxleHardpoints:
    left = default_front_left()
    return AxleHardpoints(left=left, right=mirror_corner(left))


def default_rear_axle() -> AxleHardpoints:
    """Illustrative rear — same pattern as front for solver validation."""
    return default_front_axle()


def axle_roll_center(
    axle: AxleHardpoints,
    z_left: float,
    z_right: float,
) -> tuple[float, float, float]:
    """
    RC height for an axle with independent L/R wheel travel.

    Uses mean of left-side and right-side constructions so single-wheel
    bump remains finite and symmetric bump is consistent.
    """
    hp_l = _shift_outer_vertical(axle.left, z_left)
    hp_r = _shift_outer_vertical(axle.right, z_right)
    rc_l, icy_l, icz_l = roll_center_height(hp_l)
    rc_r, icy_r, icz_r = roll_center_height(hp_r)

    if np.isfinite(rc_l) and np.isfinite(rc_r):
        rc = 0.5 * (rc_l + rc_r)
    elif np.isfinite(rc_l):
        rc = rc_l
    elif np.isfinite(rc_r):
        rc = rc_r
    else:
        rc = float("nan")

    # representative IC: average when both finite
    if np.isfinite(icy_l) and np.isfinite(icy_r):
        icy = 0.5 * (icy_l + icy_r)
        icz = 0.5 * (icz_l + icz_r)
    elif np.isfinite(icy_l):
        icy, icz = icy_l, icz_l
    else:
        icy, icz = icy_r, icz_r

    return float(rc), float(icy), float(icz)


class RollCenterModel:
    """
    Stateful front/rear roll-center evaluator.

    Static heights cached at construction for migration diagnostics.
    """

    def __init__(
        self,
        front: AxleHardpoints = None,
        rear: AxleHardpoints = None,
    ):
        self.front = front or default_front_axle()
        self.rear = rear or default_rear_axle()
        self.rc_front_static, self.ic_f_y0, self.ic_f_z0 = axle_roll_center(
            self.front, 0.0, 0.0
        )
        self.rc_rear_static, self.ic_r_y0, self.ic_r_z0 = axle_roll_center(
            self.rear, 0.0, 0.0
        )
        self.last = RollCenterState(
            rc_front_z=self.rc_front_static,
            rc_rear_z=self.rc_rear_static,
            rc_front_static_z=self.rc_front_static,
            rc_rear_static_z=self.rc_rear_static,
            ic_front_y=self.ic_f_y0,
            ic_front_z=self.ic_f_z0,
            ic_rear_y=self.ic_r_y0,
            ic_rear_z=self.ic_r_z0,
        )

    def reset(self):
        self.last = RollCenterState(
            rc_front_z=self.rc_front_static,
            rc_rear_z=self.rc_rear_static,
            rc_front_static_z=self.rc_front_static,
            rc_rear_static_z=self.rc_rear_static,
            ic_front_y=self.ic_f_y0,
            ic_front_z=self.ic_f_z0,
            ic_rear_y=self.ic_r_y0,
            ic_rear_z=self.ic_r_z0,
        )

    def evaluate(self, wheel_travel: np.ndarray) -> RollCenterState:
        """
        wheel_travel: (4,) [FL, FR, RL, RR], + = compression.
        """
        z = np.asarray(wheel_travel, dtype=float).reshape(4)
        rc_f, icy_f, icz_f = axle_roll_center(self.front, z[0], z[1])
        rc_r, icy_r, icz_r = axle_roll_center(self.rear, z[2], z[3])
        self.last = RollCenterState(
            wheel_travel=z.copy(),
            rc_front_z=rc_f,
            rc_rear_z=rc_r,
            rc_front_static_z=self.rc_front_static,
            rc_rear_static_z=self.rc_rear_static,
            rc_front_migration=rc_f - self.rc_front_static
            if np.isfinite(rc_f) else float("nan"),
            rc_rear_migration=rc_r - self.rc_rear_static
            if np.isfinite(rc_r) else float("nan"),
            ic_front_y=icy_f,
            ic_front_z=icz_f,
            ic_rear_y=icy_r,
            ic_rear_z=icz_r,
        )
        return self.last
