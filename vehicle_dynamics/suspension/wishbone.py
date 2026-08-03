"""
Double-wishbone kinematic quantities at a fixed configuration.

Stage 1 (Phase 6.0): static geometry at design (or prescribed) hardpoints.
Wheel travel / iteration comes in later phases.
"""

from __future__ import annotations

import numpy as np
from .hardpoints import WishboneHardpoints, Point3
from .geometry import (
    instant_center_yz, average_inner, unit, line_intersect_2d,
)
from .result import GeometryResult


def _steer_axis(hp: WishboneHardpoints) -> np.ndarray:
    """Unit vector along kingpin (upper outer → lower outer, or reverse)."""
    v = hp.upper_outer.as_array() - hp.lower_outer.as_array()
    return unit(v)


def kingpin_inclination_deg(hp: WishboneHardpoints) -> float:
    """KPI: angle of steer axis from vertical in front view (YZ)."""
    axis = _steer_axis(hp)
    # projection on YZ: (ay, az)
    ay, az = axis[1], axis[2]
    # angle from +z
    return float(np.degrees(np.arctan2(abs(ay), abs(az) + 1e-12)))


def caster_deg(hp: WishboneHardpoints) -> float:
    """Caster: angle of steer axis from vertical in side view (XZ)."""
    axis = _steer_axis(hp)
    ax, az = axis[0], axis[2]
    return float(np.degrees(np.arctan2(ax, abs(az) + 1e-12)))


def scrub_radius(hp: WishboneHardpoints) -> float:
    """
    Lateral distance at ground between contact patch and intersection of
    steer axis with ground plane z=0.
    """
    lo = hp.lower_outer.as_array()
    up = hp.upper_outer.as_array()
    axis = unit(up - lo)
    # line: lo + t * axis hits z=0
    if abs(axis[2]) < 1e-12:
        return float("nan")
    t = -lo[2] / axis[2]
    hit = lo + t * axis
    cp = hp.contact_patch.as_array()
    # signed scrub: hit_y - cp_y (left side typically negative if axis tilts in)
    return float(hit[1] - cp[1])


def trail(hp: WishboneHardpoints) -> float:
    """Longitudinal trail: contact patch x relative to steer-axis ground hit."""
    lo = hp.lower_outer.as_array()
    up = hp.upper_outer.as_array()
    axis = unit(up - lo)
    if abs(axis[2]) < 1e-12:
        return float("nan")
    t = -lo[2] / axis[2]
    hit = lo + t * axis
    cp = hp.contact_patch.as_array()
    return float(cp[0] - hit[0])


def camber_deg(hp: WishboneHardpoints) -> float:
    """
    Camber from wheel center relative to upright (lower→upper outer).
    Positive camber = top of wheel outward (+y for left wheel if upright tilts out).
    Simplified: angle of upright in YZ from vertical.
    """
    upright = hp.upper_outer.as_array() - hp.lower_outer.as_array()
    # for left wheel, outward is +y
    ay, az = upright[1], upright[2]
    # camber ≈ angle from vertical; sign: positive if top outward
    return float(np.degrees(np.arctan2(ay, abs(az) + 1e-12)))


def toe_deg(hp: WishboneHardpoints) -> float:
    """
    Toe from tierod orientation in plan view (XY).
    Positive toe-in: front of wheel points inward (−y for left).
    Approximate using tierod_outer − wheel_center direction projected to XY.
    """
    # wheel pointing direction approx from lower_outer to a forward point
    # Use tierod: steering arm lateral relative to wheel center
    arm = hp.tierod_outer.as_array() - hp.wheel_center.as_array()
    # toe angle of wheel plane: use vector from lower_outer to upper_outer
    # and tierod to infer rotation about vertical — simplified static toe
    # from plan projection of lower arm outer vs wheel center x-offset
    wc = hp.wheel_center.as_array()
    lo = hp.lower_outer.as_array()
    # direction wheel faces (x-forward component vs y)
    # Static design toe from tierod length geometry:
    dx = hp.tierod_outer.x - hp.tierod_inner.x
    dy = hp.tierod_outer.y - hp.tierod_inner.y
    # not pure toe; better: angle of line wheel_center → point on rim front
    # Use upright plane normal projected: (upper-lower) × (tierod_outer-lower)
    u = hp.upper_outer.as_array() - lo
    t = hp.tierod_outer.as_array() - lo
    n = np.cross(u, t)
    if abs(n[2]) < 1e-12 and np.linalg.norm(n[:2]) < 1e-12:
        return 0.0
    # heading of wheel plane intersection with ground: perpendicular to n in XY
    # wheel forward direction ≈ (-n_y, n_x) or similar
    fwd = np.array([-n[1], n[0]])
    if np.linalg.norm(fwd) < 1e-12:
        return 0.0
    fwd = fwd / np.linalg.norm(fwd)
    # angle from +x
    return float(np.degrees(np.arctan2(fwd[1], fwd[0])))


def roll_center_front_view(hp: WishboneHardpoints) -> tuple[float, float] | None:
    """
    Instant center in YZ, then roll center is intersection of IC–contact_patch
    line with vehicle centerline y=0 (classic 2D construction).
    Returns (y, z) of roll center; y should be ≈ 0.
    """
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    ic = instant_center_yz(
        ui.y, ui.z,
        hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z,
        hp.lower_outer.y, hp.lower_outer.z,
    )
    if ic is None:
        return None
    ic_y, ic_z = ic
    cp = hp.contact_patch
    # line IC → CP, intersect y=0
    dy = cp.y - ic_y
    dz = cp.z - ic_z
    if abs(dy) < 1e-12:
        return 0.0, ic_z  # already on centerline
    t = (0.0 - ic_y) / dy
    rc_z = ic_z + t * dz
    return 0.0, float(rc_z)


def swing_arm_length(hp: WishboneHardpoints) -> float | None:
    """Distance from contact patch to YZ instant center."""
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    ic = instant_center_yz(
        ui.y, ui.z, hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z, hp.lower_outer.y, hp.lower_outer.z,
    )
    if ic is None:
        return None
    cp = hp.contact_patch
    return float(np.hypot(ic[0] - cp.y, ic[1] - cp.z))


def analyze(hp: WishboneHardpoints) -> GeometryResult:
    """Full static geometry analysis for one corner."""
    rc = roll_center_front_view(hp)
    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)
    ic = instant_center_yz(
        ui.y, ui.z, hp.upper_outer.y, hp.upper_outer.z,
        li.y, li.z, hp.lower_outer.y, hp.lower_outer.z,
    )
    return GeometryResult(
        camber_deg=camber_deg(hp),
        toe_deg=toe_deg(hp),
        caster_deg=caster_deg(hp),
        kpi_deg=kingpin_inclination_deg(hp),
        scrub_radius=scrub_radius(hp),
        trail=trail(hp),
        roll_center_z=rc[1] if rc else float("nan"),
        instant_center_y=ic[0] if ic else float("nan"),
        instant_center_z=ic[1] if ic else float("nan"),
        swing_arm_length=swing_arm_length(hp) or float("nan"),
        upper_arm_length=float(np.linalg.norm(
            hp.upper_outer.as_array() - ui.as_array()
        )),
        lower_arm_length=float(np.linalg.norm(
            hp.lower_outer.as_array() - li.as_array()
        )),
    )
