"""Class-A surface quality helpers."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .continuity import ContinuityAnalyzer, ContinuityResult


@dataclass
class ClassAReport:
    continuity: ContinuityResult | None
    fairness: dict[str, float]
    reflection_quality: float
    violations: list[str]

    @property
    def passes(self) -> bool:
        return len(self.violations) == 0


def analyze_class_a(surface, curve_a=None, curve_b=None) -> ClassAReport:
    analyzer = ContinuityAnalyzer()
    cont = None
    violations = []
    if curve_a is not None and curve_b is not None:
        cont = analyzer.analyze_curves(curve_a, curve_b)
        if not cont.g0:
            violations.append("G0 position discontinuity")
        if not cont.g1:
            violations.append("G1 tangent discontinuity")
        if not cont.g2:
            violations.append("G2 curvature discontinuity")
        if not cont.g3:
            violations.append("G3 curvature-flow discontinuity")
    fairness = analyzer.analyze_surface_fairness(surface)
    reflection = fairness.get("fairness_score", 0.0)
    if reflection < 0.5:
        violations.append("poor surface fairness / reflection quality")
    return ClassAReport(continuity=cont, fairness=fairness, reflection_quality=reflection, violations=violations)
