"""
Phase 6.6 – Geometry at prescribed wheel travel.

Length-preserving YZ kinematics with exact design hardpoints at z=0.
"""

from __future__ import annotations

import numpy as np
from .hardpoints import WishboneHardpoints, Point3, default_front_left
from .wishbone import analyze
from .result import GeometryResult
from .geometry import average_inner


def _solve_y_on_circle(cy: float, cz: float, radius: float, z: float, y_hint: float) -> float:
    dz = z - cz
    r2 = radius * radius - dz * dz
    if r2 < 0:
        return float(y_hint)
    dy = np.sqrt(r2)
    y1, y2 = cy + dy, cy - dy
    return float(y1 if abs(y1 - y_hint) <= abs(y2 - y_hint) else y2)


def displace_kinematic(hp: WishboneHardpoints, wheel_travel: float) -> WishboneHardpoints:
    """
    Approximate double-wishbone motion in the front view (YZ).
    wheel_travel > 0: compression (wheel up relative to body).
    At travel ≈ 0 returns the original hardpoints unchanged (Phase 6.0 regression).
    """
    dz = float(wheel_travel)
    if abs(dz) < 1e-12:
        return hp

    ui = average_inner(hp.upper_front, hp.upper_rear)
    li = average_inner(hp.lower_front, hp.lower_rear)

    uo = hp.upper_outer.as_array()
    lo = hp.lower_outer.as_array()
    Lu = float(np.hypot(uo[1] - ui.y, uo[2] - ui.z))
    Ll = float(np.hypot(lo[1] - li.y, lo[2] - li.z))
    upright_len = float(np.linalg.norm(uo - lo))

    lo_z = lo[2] + dz
    lo_y = _solve_y_on_circle(li.y, li.z, Ll, lo_z, lo[1])

    best = None
    best_err = 1e9
    for ang in np.linspace(-np.pi, np.pi, 721):
        uy = ui.y + Lu * np.cos(ang)
        uz = ui.z + Lu * np.sin(ang)
        dist = np.hypot(uy - lo_y, uz - lo_z)
        err = abs(dist - upright_len)
        err += 0.05 * np.hypot(uy - uo[1], uz - uo[2])
        if err < best_err:
            best_err = err
            best = (uy, uz)
    uy, uz = best if best is not None else (uo[1], uo[2] + dz)

    def P(x, y, z):
        return Point3(float(x), float(y), float(z))

    d_y = 0.5 * ((uy - uo[1]) + (lo_y - lo[1]))
    d_z = 0.5 * ((uz - uo[2]) + (lo_z - lo[2]))

    return WishboneHardpoints(
        upper_front=hp.upper_front,
        upper_rear=hp.upper_rear,
        upper_outer=P(uo[0], uy, uz),
        lower_front=hp.lower_front,
        lower_rear=hp.lower_rear,
        lower_outer=P(lo[0], lo_y, lo_z),
        tierod_inner=hp.tierod_inner,
        tierod_outer=P(
            hp.tierod_outer.x,
            hp.tierod_outer.y + d_y,
            hp.tierod_outer.z + d_z,
        ),
        wheel_center=P(
            hp.wheel_center.x,
            hp.wheel_center.y + d_y,
            hp.wheel_center.z + d_z,
        ),
        contact_patch=Point3(hp.contact_patch.x, hp.contact_patch.y + d_y, 0.0),
    )


def solve_at_travel(
    hp: WishboneHardpoints,
    wheel_travel: float,
) -> GeometryResult:
    displaced = displace_kinematic(hp, float(wheel_travel))
    return analyze(displaced)


def solve_static(hp: WishboneHardpoints = None) -> GeometryResult:
    return analyze(hp or default_front_left())
