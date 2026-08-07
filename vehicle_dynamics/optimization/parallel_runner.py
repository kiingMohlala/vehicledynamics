"""Parallel batch evaluation via multiprocessing or thread pool."""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Callable
import numpy as np

from .parameter_space import ParameterSpace
from .batch_runner import BatchResult, default_evaluator
from .constraints import Constraint, enforce
from .objective_functions import Objective, lap_time_objective


def _eval_one(args: tuple) -> tuple[int, dict, dict, bool, float]:
    idx, design, constraints_ok, obj_vals = args
    # constraints_ok precomputed; obj_vals is the output dict score components
    return idx, design, obj_vals, constraints_ok, float(obj_vals.get("_score", 1e9))


@dataclass
class ParallelRunner:
    evaluator: Callable[[dict[str, float]], dict[str, Any]] | None = None
    constraints: list[Constraint] = field(default_factory=list)
    objectives: list[Objective] = field(default_factory=lambda: [lap_time_objective()])
    workers: int = 2
    backend: str = "thread"  # thread | process

    def __post_init__(self) -> None:
        self.evaluator = self.evaluator or default_evaluator

    def run(self, space: ParameterSpace) -> BatchResult:
        designs = space.as_dicts()
        n = len(designs)
        outputs: list[dict[str, Any] | None] = [None] * n
        feasible = [True] * n
        scores = [1e9] * n

        def work(i: int, d: dict[str, float]):
            ok, _ = enforce(d, self.constraints)
            if not ok:
                out = {"lap_time": 1e9, "energy": 1e9, "feasible": False, "design": d}
                score = 1e9
            else:
                out = self.evaluator(d)
                score = sum(o.weight * o.evaluate(out) for o in self.objectives)
            return i, d, out, ok, score

        Executor = ThreadPoolExecutor if self.backend == "thread" else ProcessPoolExecutor
        with Executor(max_workers=max(1, self.workers)) as ex:
            futs = [ex.submit(work, i, d) for i, d in enumerate(designs)]
            for fut in as_completed(futs):
                i, d, out, ok, score = fut.result()
                outputs[i] = out
                feasible[i] = ok
                scores[i] = score

        best = int(np.argmin(scores)) if scores else 0
        return BatchResult(
            designs=designs,
            outputs=[o or {} for o in outputs],
            feasible=feasible,
            objective_values=scores,
            best_index=best,
            objective_name=self.objectives[0].name if self.objectives else "obj",
        )
