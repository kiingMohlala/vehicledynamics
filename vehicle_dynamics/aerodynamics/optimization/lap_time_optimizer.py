"""Single-objective lap-time optimizer (random search + local refinement)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .design_variables import DesignVector, DesignBounds, default_bounds
from .objective_functions import evaluate_objectives
from .constraints import evaluate_constraints


@dataclass
class LapTimeModel:
    speed_corner: float = 35.0
    speed_straight: float = 60.0

    def evaluate(self, design: DesignVector) -> float:
        oc = evaluate_objectives(design, speed=self.speed_corner)
        os_ = evaluate_objectives(design, speed=self.speed_straight)
        # Weighted blend corner/straight
        return 0.55 * oc.lap_time + 0.45 * os_.lap_time


@dataclass
class LapTimeResult:
    best_design: DesignVector
    best_time: float
    history: list[float]


def optimize_lap_time(
    bounds: DesignBounds | None = None,
    *,
    n_samples: int = 80,
    n_refine: int = 30,
    seed: int = 0,
) -> LapTimeResult:
    bounds = bounds or default_bounds()
    rng = np.random.default_rng(seed)
    model = LapTimeModel()
    lo, hi = bounds.low.as_array(), bounds.high.as_array()

    best_d = bounds.random(rng)
    best_t = model.evaluate(best_d)
    hist = [best_t]

    for _ in range(n_samples):
        d = bounds.random(rng)
        o = evaluate_objectives(d, speed=50.0)
        ok, _ = evaluate_constraints(d, o.drag, o.front_balance)
        if not ok:
            continue
        t = model.evaluate(d)
        if t < best_t:
            best_t, best_d = t, d
        hist.append(best_t)

    # Local random refinement
    x = best_d.as_array()
    for k in range(n_refine):
        step = 0.1 * (1.0 - k / n_refine) * (hi - lo)
        cand = np.clip(x + rng.normal(0, 1, size=x.shape) * step, lo, hi)
        d = DesignVector.from_array(cand)
        o = evaluate_objectives(d, speed=50.0)
        ok, _ = evaluate_constraints(d, o.drag, o.front_balance)
        if not ok:
            continue
        t = model.evaluate(d)
        if t < best_t:
            best_t, best_d, x = t, d, cand
        hist.append(best_t)

    return LapTimeResult(best_design=best_d, best_time=best_t, history=hist)
