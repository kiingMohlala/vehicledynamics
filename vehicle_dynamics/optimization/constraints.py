"""Design constraints."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class Constraint:
    name: str
    fn: Callable[[dict[str, float]], bool]
    description: str = ""

    def satisfied(self, design: dict[str, float]) -> bool:
        return bool(self.fn(design))


def bound_constraint(name: str, low: float, high: float) -> Constraint:
    return Constraint(
        name=f"{name}_bounds",
        fn=lambda d, n=name, lo=low, hi=high: lo <= float(d.get(n, lo)) <= hi,
        description=f"{low} ≤ {name} ≤ {high}",
    )


def ratio_constraint(a: str, b: str, max_ratio: float) -> Constraint:
    return Constraint(
        name=f"{a}_over_{b}",
        fn=lambda d, aa=a, bb=b, r=max_ratio: abs(float(d.get(bb, 1.0))) < 1e-12
        or abs(float(d.get(aa, 0.0)) / float(d.get(bb, 1.0))) <= r,
        description=f"|{a}/{b}| ≤ {max_ratio}",
    )


def enforce(design: dict[str, float], constraints: list[Constraint]) -> tuple[bool, list[str]]:
    failed = [c.name for c in constraints if not c.satisfied(design)]
    return len(failed) == 0, failed
