"""Pareto front extraction for multi-objective results."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass
class ParetoFront:
    indices: list[int]
    points: list[dict[str, float]]  # objective values
    designs: list[dict[str, float]]

    @property
    def size(self) -> int:
        return len(self.indices)


def dominates(a: np.ndarray, b: np.ndarray) -> bool:
    """a dominates b if a is <= b in all objs and < in at least one (minimization)."""
    return bool(np.all(a <= b) and np.any(a < b))


def pareto_front(
    outputs: list[dict[str, Any]],
    designs: list[dict[str, float]],
    objectives: list[str],
    maximize: list[str] | None = None,
) -> ParetoFront:
    maximize = set(maximize or [])
    n = len(outputs)
    if n == 0:
        return ParetoFront([], [], [])
    mat = np.zeros((n, len(objectives)))
    for i, o in enumerate(outputs):
        for j, name in enumerate(objectives):
            val = float(o.get(name, 0.0))
            mat[i, j] = -val if name in maximize else val

    is_pareto = np.ones(n, dtype=bool)
    for i in range(n):
        if not is_pareto[i]:
            continue
        for j in range(n):
            if i == j or not is_pareto[j]:
                continue
            if dominates(mat[j], mat[i]):
                is_pareto[i] = False
                break
    idxs = [i for i in range(n) if is_pareto[i]]
    points = [{name: float(outputs[i].get(name, 0.0)) for name in objectives} for i in idxs]
    des = [designs[i] for i in idxs]
    return ParetoFront(indices=idxs, points=points, designs=des)
