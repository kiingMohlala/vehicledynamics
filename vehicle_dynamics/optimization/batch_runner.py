"""Sequential batch evaluation of design samples."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable
import numpy as np

from .parameter_space import ParameterSpace
from .constraints import Constraint, enforce
from .objective_functions import Objective, lap_time_objective


Evaluator = Callable[[dict[str, float]], dict[str, Any]]


def default_evaluator(design: dict[str, float]) -> dict[str, Any]:
    """
    Lightweight surrogate physics for DOE testing / smoke runs.
    Maps spring/wing-like parameters to a synthetic lap_time-like metric.
    """
    # Generic response surface (smooth, deterministic)
    vals = list(design.values())
    if not vals:
        return {"lap_time": 100.0, "energy": 1.0, "top_speed": 50.0, "rms_ax": 1.0}
    x = np.array(vals, dtype=float)
    # normalize roughly
    z = (x - np.mean(x)) / (np.std(x) + 1e-9)
    lap = 90.0 + 0.5 * float(np.sum(z ** 2)) + 0.1 * float(z[0] if len(z) else 0)
    energy = 10.0 + 0.2 * float(np.sum(np.abs(z)))
    top = 60.0 - 0.05 * lap
    return {
        "lap_time": lap,
        "energy": energy,
        "top_speed": top,
        "rms_ax": 0.5 + 0.01 * abs(lap - 90),
        "design": design,
    }


@dataclass
class BatchResult:
    designs: list[dict[str, float]]
    outputs: list[dict[str, Any]]
    feasible: list[bool]
    objective_values: list[float]
    best_index: int = 0
    objective_name: str = "lap_time"

    @property
    def best_design(self) -> dict[str, float]:
        return self.designs[self.best_index] if self.designs else {}

    @property
    def best_output(self) -> dict[str, Any]:
        return self.outputs[self.best_index] if self.outputs else {}

    @property
    def n(self) -> int:
        return len(self.designs)


@dataclass
class BatchRunner:
    evaluator: Evaluator | None = None
    constraints: list[Constraint] = field(default_factory=list)
    objectives: list[Objective] = field(default_factory=lambda: [lap_time_objective()])

    def __post_init__(self) -> None:
        self.evaluator = self.evaluator or default_evaluator

    def run(self, space: ParameterSpace) -> BatchResult:
        designs, outputs, feasible, objs = [], [], [], []
        for d in space:
            ok, _ = enforce(d, self.constraints)
            feasible.append(ok)
            if ok:
                out = self.evaluator(d)
            else:
                out = {"lap_time": 1e9, "energy": 1e9, "feasible": False, "design": d}
            designs.append(d)
            outputs.append(out)
            objs.append(sum(o.weight * o.evaluate(out) for o in self.objectives))
        best = int(np.argmin(objs)) if objs else 0
        return BatchResult(
            designs=designs,
            outputs=outputs,
            feasible=feasible,
            objective_values=objs,
            best_index=best,
            objective_name=self.objectives[0].name if self.objectives else "obj",
        )
