"""Tolerance stack-up and fit analysis."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Tolerance:
    name: str
    nominal: float
    plus: float
    minus: float

    @property
    def span(self) -> float:
        return self.plus + self.minus


@dataclass
class StackResult:
    nominal: float
    max: float
    min: float
    span: float
    fit: str  # clearance | transition | interference | n/a


def stack_up(tolerances: list[Tolerance], signs: list[int] | None = None) -> StackResult:
    """Worst-case linear stack-up."""
    if signs is None:
        signs = [1] * len(tolerances)
    nom = sum(s * t.nominal for s, t in zip(signs, tolerances))
    # max: for +sign use plus, for -sign use minus contribution inverted
    mx = sum((t.nominal + t.plus) if s > 0 else (t.nominal - t.minus) * s for s, t in zip(signs, tolerances))
    # simpler worst case
    mx = sum(s * t.nominal + abs(s) * t.plus for s, t in zip(signs, tolerances))
    mn = sum(s * t.nominal - abs(s) * t.minus for s, t in zip(signs, tolerances))
    span = mx - mn
    return StackResult(nom, mx, mn, span, "n/a")


def clearance_analysis(hole: Tolerance, shaft: Tolerance) -> StackResult:
    """Fit between hole and shaft (radial)."""
    # clearance = hole - shaft
    cmax = (hole.nominal + hole.plus) - (shaft.nominal - shaft.minus)
    cmin = (hole.nominal - hole.minus) - (shaft.nominal + shaft.plus)
    nom = hole.nominal - shaft.nominal
    if cmin > 0:
        fit = "clearance"
    elif cmax < 0:
        fit = "interference"
    else:
        fit = "transition"
    return StackResult(nom, cmax, cmin, cmax - cmin, fit)
