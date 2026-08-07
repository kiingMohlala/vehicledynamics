"""Aggregate analysis helpers for batch results."""
from __future__ import annotations

from typing import Any
import numpy as np

from .batch_runner import BatchResult
from .pareto_analysis import pareto_front, ParetoFront
from .sensitivity import correlation_sensitivity, SensitivityResult


def summarize(result: BatchResult) -> dict[str, Any]:
    objs = np.array(result.objective_values, dtype=float)
    return {
        "n": result.n,
        "n_feasible": int(sum(result.feasible)),
        "best_index": result.best_index,
        "best_objective": float(objs[result.best_index]) if len(objs) else None,
        "mean_objective": float(np.mean(objs)) if len(objs) else None,
        "std_objective": float(np.std(objs)) if len(objs) else None,
        "best_design": result.best_design,
    }


def analyze(
    result: BatchResult,
    pareto_objectives: list[str] | None = None,
) -> dict[str, Any]:
    pareto_objectives = pareto_objectives or ["lap_time", "energy"]
    pf = pareto_front(result.outputs, result.designs, pareto_objectives, maximize=[])
    sens = correlation_sensitivity(result.designs, result.outputs, objective_key=pareto_objectives[0])
    return {
        "summary": summarize(result),
        "pareto": pf,
        "sensitivity": sens,
    }
