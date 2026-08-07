"""One-at-a-time and correlation-based sensitivity analysis."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable
import numpy as np

from .design_variables import DesignVariable
from .batch_runner import default_evaluator


@dataclass
class SensitivityResult:
    rankings: list[tuple[str, float]]  # (name, score) high = more sensitive
    effects: dict[str, float]

    def top(self, k: int = 3) -> list[tuple[str, float]]:
        return self.rankings[:k]


def local_sensitivity(
    variables: list[DesignVariable],
    evaluator: Callable[[dict[str, float]], dict[str, Any]] | None = None,
    objective_key: str = "lap_time",
    delta_frac: float = 0.05,
) -> SensitivityResult:
    evaluator = evaluator or default_evaluator
    base = {v.name: 0.5 * (v.low + v.high) for v in variables}
    base_out = float(evaluator(base).get(objective_key, 0.0))
    effects = {}
    for v in variables:
        span = v.high - v.low
        d = delta_frac * span
        hi = dict(base)
        lo = dict(base)
        hi[v.name] = v.clip(base[v.name] + d)
        lo[v.name] = v.clip(base[v.name] - d)
        y_hi = float(evaluator(hi).get(objective_key, 0.0))
        y_lo = float(evaluator(lo).get(objective_key, 0.0))
        effects[v.name] = abs(y_hi - y_lo) / (2 * delta_frac + 1e-15)
    rankings = sorted(effects.items(), key=lambda kv: kv[1], reverse=True)
    return SensitivityResult(rankings=rankings, effects=effects)


def correlation_sensitivity(
    designs: list[dict[str, float]],
    outputs: list[dict[str, Any]],
    objective_key: str = "lap_time",
) -> SensitivityResult:
    if not designs:
        return SensitivityResult([], {})
    names = list(designs[0].keys())
    Y = np.array([float(o.get(objective_key, 0.0)) for o in outputs])
    effects = {}
    for n in names:
        X = np.array([float(d[n]) for d in designs])
        if np.std(X) < 1e-15 or np.std(Y) < 1e-15:
            effects[n] = 0.0
        else:
            effects[n] = float(abs(np.corrcoef(X, Y)[0, 1]))
    rankings = sorted(effects.items(), key=lambda kv: kv[1], reverse=True)
    return SensitivityResult(rankings=rankings, effects=effects)
