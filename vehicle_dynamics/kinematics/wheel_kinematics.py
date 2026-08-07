"""
Kinematic wheel motion from hardpoints.

Uses a simplified geometric model:
  - For double wishbone: rotate arms about chassis pickups to enforce
    approximate constant arm lengths while moving outer ball joints with travel.
  - For MacPherson: LCA arc + strut length constraint (simplified).

This is an engineering kinematics approximation suitable for alignment curves,
not a full multi-body constraint Newton solve.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .hardpoints import HardpointSet
from .alignment import (
    camber_from_upright, toe_from_heading, caster_from_kingpin,
    kpi_from_kingpin, scrub_radius, mechanical_trail,
)
from .instant_center import front_view_ic, side_view_ic
from .roll_center import roll_center_height


@dataclass
class CornerState:
    wheel_center: np.ndarray
    contact: np.ndarray
    camber: float
    toe: float
    caster: float
    kpi: float
    scrub: float
    trail: float
    travel: float
    ic_front_view: np.ndarray
    ic_side_view: np.ndarray
    roll_center_z: float


def _move_point_vertical(p: np.ndarray, dz: float) -> np.ndarray:
    q = p.copy()
    q[2] += dz
    return q


def solve_double_wishbone_corner(hp: HardpointSet, travel: float) -> CornerState:
    """
    travel: vertical wheel-center displacement relative to design (m).
    Outer ball joints and wheel center move primarily in Z; lateral/longitudinal
    adjustments approximate arm-length constraint via radial projection.
    """
    wc0 = hp.get("wheel_center")
    lca_o0 = hp.get("LCA_outer")
    uca_o0 = hp.get("UCA_outer")
    lca_i = 0.5 * (hp.get("LCA_front") + hp.get("LCA_rear"))
    uca_i = 0.5 * (hp.get("UCA_front") + hp.get("UCA_rear"))

    # target wheel center
    wc = _move_point_vertical(wc0, travel)

    # project outer joints to maintain arm lengths (spherical approx in 3D)
    def project_length(inner, outer0, target_hint):
        L = np.linalg.norm(outer0 - inner)
        direction = target_hint - inner
        n = np.linalg.norm(direction)
        if n < 1e-12:
            return outer0.copy()
        return inner + direction / n * L

    lca_o = project_length(lca_i, lca_o0, _move_point_vertical(lca_o0, travel))
    uca_o = project_length(uca_i, uca_o0, _move_point_vertical(uca_o0, travel * 0.9))

    # upright from outer ball joints
    contact = wc.copy()
    contact[2] = wc[2] - abs(wc0[2] - (hp.points.get("contact", np.array([wc0[0], wc0[1], -0.32]))[2] if "contact" in hp.points else 0.32))
    # default tire radius 0.32 if no contact point
    if "contact" not in hp.points:
        contact = wc.copy()
        contact[2] = wc[2] - 0.32

    camber = float(np.arctan2(uca_o[1] - lca_o[1], uca_o[2] - lca_o[2]) - np.arctan2(
        uca_o0[1] - lca_o0[1], uca_o0[2] - lca_o0[2]
    ))
    # static camber offset from upright
    camber += camber_from_upright(wc, contact) * 0.0  # keep incremental

    # toe from tierod if present
    toe = 0.0
    if "tierod_outer" in hp.points and "tierod_inner" in hp.points:
        tr_o0 = hp.get("tierod_outer")
        tr_i = hp.get("tierod_inner")
        tr_o = project_length(tr_i, tr_o0, _move_point_vertical(tr_o0, travel * 0.5))
        heading0 = tr_o0[:2] - wc0[:2]
        heading = tr_o[:2] - wc[:2]
        toe = float(np.arctan2(heading[1], heading[0]) - np.arctan2(heading0[1], heading0[0]))

    upper = uca_o
    lower = lca_o
    caster = caster_from_kingpin(upper, lower)
    kpi = kpi_from_kingpin(upper, lower)
    scrub = scrub_radius(wc, contact, upper, lower)
    trail = mechanical_trail(wc, contact, upper, lower)

    ic_fv = front_view_ic(lca_i[[1, 2]], lca_o[[1, 2]], uca_i[[1, 2]], uca_o[[1, 2]])
    # side view using front/rear pickups
    lca_f, lca_r = hp.get("LCA_front"), hp.get("LCA_rear")
    uca_f, uca_r = hp.get("UCA_front"), hp.get("UCA_rear")
    ic_sv = side_view_ic(lca_f[[0, 2]], lca_r[[0, 2]], uca_f[[0, 2]], uca_r[[0, 2]])
    rc_z = roll_center_height(ic_fv, contact[[1, 2]])

    return CornerState(
        wheel_center=wc, contact=contact, camber=camber, toe=toe,
        caster=caster, kpi=kpi, scrub=scrub, trail=trail, travel=travel,
        ic_front_view=ic_fv, ic_side_view=ic_sv, roll_center_z=rc_z,
    )


def solve_macpherson_corner(hp: HardpointSet, travel: float) -> CornerState:
    wc0 = hp.get("wheel_center")
    lca_o0 = hp.get("LCA_outer")
    lca_i = 0.5 * (hp.get("LCA_front") + hp.get("LCA_rear"))
    strut_u = hp.get("strut_upper")
    strut_l0 = hp.get("strut_lower")

    wc = wc0.copy(); wc[2] += travel
    L = np.linalg.norm(lca_o0 - lca_i)
    hint = lca_o0.copy(); hint[2] += travel
    direction = hint - lca_i
    lca_o = lca_i + direction / (np.linalg.norm(direction) + 1e-15) * L

    strut_l = strut_l0.copy(); strut_l[2] += travel
    contact = wc.copy(); contact[2] = wc[2] - 0.32

    camber = float(np.arctan2((strut_u[1] - lca_o[1]), (strut_u[2] - lca_o[2])) -
                   np.arctan2((strut_u[1] - lca_o0[1]), (strut_u[2] - lca_o0[2])))
    toe = 0.0
    if "tierod_outer" in hp.points:
        tr_o0 = hp.get("tierod_outer")
        tr_i = hp.get("tierod_inner")
        tr_L = np.linalg.norm(tr_o0 - tr_i)
        hint_t = tr_o0.copy(); hint_t[2] += travel * 0.5
        tr_o = tr_i + (hint_t - tr_i) / (np.linalg.norm(hint_t - tr_i) + 1e-15) * tr_L
        toe = float(np.arctan2(tr_o[1] - wc[1], tr_o[0] - wc[0]) -
                    np.arctan2(tr_o0[1] - wc0[1], tr_o0[0] - wc0[0]))

    caster = caster_from_kingpin(strut_u, lca_o)
    kpi = kpi_from_kingpin(strut_u, lca_o)
    scrub = scrub_radius(wc, contact, strut_u, lca_o)
    trail = mechanical_trail(wc, contact, strut_u, lca_o)

    # MacPherson IC: LCA line + strut axis approx perpendicular
    ic_fv = front_view_ic(lca_i[[1, 2]], lca_o[[1, 2]], strut_u[[1, 2]], strut_l[[1, 2]])
    ic_sv = np.array([wc[0], 0.0])
    rc_z = roll_center_height(ic_fv, contact[[1, 2]])

    return CornerState(
        wheel_center=wc, contact=contact, camber=camber, toe=toe,
        caster=caster, kpi=kpi, scrub=scrub, trail=trail, travel=travel,
        ic_front_view=ic_fv, ic_side_view=ic_sv, roll_center_z=rc_z,
    )
