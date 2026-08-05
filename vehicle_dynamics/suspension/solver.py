"""
Phase 6.0 – Static suspension geometry solver.

Independent of vehicle dynamics. Input = hardpoints; output = GeometryResult.
"""

from __future__ import annotations

from .hardpoints import WishboneHardpoints, default_front_left, mirror_corner
from .wishbone import analyze
from .result import GeometryResult


class SuspensionGeometrySolver:
    def __init__(self, hardpoints: WishboneHardpoints = None):
        self.hp = hardpoints or default_front_left()

    def solve(self) -> GeometryResult:
        return analyze(self.hp)

    def solve_pair(self) -> tuple[GeometryResult, GeometryResult]:
        """Left (given) and mirrored right corner."""
        left = analyze(self.hp)
        right = analyze(mirror_corner(self.hp))
        return left, right
