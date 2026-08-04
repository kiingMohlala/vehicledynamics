"""
Phase 6.6 – Geometry at prescribed wheel travel.

Displaces outer upright attachments by wheel travel and re-runs the Phase 6.0
analyze() construction (IC, RC, camber, toe, KPI, caster, scrub, trail).
"""

from __future__ import annotations

from .hardpoints import WishboneHardpoints, Point3, default_front_left
from .wishbone import analyze
from .result import GeometryResult
from .roll_center import displace_corner


def solve_at_travel(
    hp: WishboneHardpoints,
    wheel_travel: float,
) -> GeometryResult:
    """
    Parameters
    ----------
    hp : design hardpoints
    wheel_travel : [m] + = compression (wheel up relative to body)

    Returns
    -------
    GeometryResult at the displaced configuration.
    """
    displaced = displace_corner(hp, float(wheel_travel))
    return analyze(displaced)


def solve_static(hp: WishboneHardpoints = None) -> GeometryResult:
    """Design position (travel = 0)."""
    return analyze(hp or default_front_left())
