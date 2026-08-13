"""Dispatch corner solves by suspension type."""
from __future__ import annotations

from .hardpoints import HardpointSet
from .wheel_kinematics import solve_double_wishbone_corner, solve_macpherson_corner, CornerState


def solve_corner(hp: HardpointSet, travel: float, suspension_type: str = "double_wishbone") -> CornerState:
    st = suspension_type.lower()
    if st in ("macpherson", "macpherson_strut"):
        return solve_macpherson_corner(hp, travel)
    if st in ("multilink", "multi-link"):
        # treat as DWB approximation if upper/lower present
        if "UCA_outer" in hp.points:
            return solve_double_wishbone_corner(hp, travel)
        return solve_macpherson_corner(hp, travel)
    # default double wishbone / pushrod / pullrod / trailing (geometry subset)
    return solve_double_wishbone_corner(hp, travel)
