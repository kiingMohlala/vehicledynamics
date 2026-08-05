"""Optimization report formatting."""

from __future__ import annotations

from .nsga2 import NSGA2Result
from .lap_time_optimizer import LapTimeResult
from .ai_explorer import ExplorationResult


def format_optimization_report(
    nsga: NSGA2Result | None = None,
    lap: LapTimeResult | None = None,
    explore: ExplorationResult | None = None,
    title: str = "Aero Optimization Report",
) -> str:
    lines = [f"=== {title} ==="]
    if nsga is not None:
        lines += [
            "",
            "NSGA-II",
            f"  Population:     {len(nsga.population)}",
            f"  Pareto size:    {len(nsga.pareto_indices)}",
            f"  Best lap (pop): {min(nsga.objectives[:, 2]):.3f} s",
            f"  Max DF (pop):   {max(nsga.objectives[:, 0]):.1f} N",
            f"  Min drag (pop): {min(nsga.objectives[:, 1]):.1f} N",
        ]
        if len(nsga.pareto_indices):
            i = nsga.pareto_indices[0]
            lines.append(
                f"  Pareto[0] DF/drag/lap: "
                f"{nsga.objectives[i, 0]:.0f} / {nsga.objectives[i, 1]:.0f} / {nsga.objectives[i, 2]:.2f}"
            )
    if lap is not None:
        lines += [
            "",
            "Lap-time optimizer",
            f"  Best time: {lap.best_time:.3f} s",
            f"  RW angle:  {lap.best_design.rear_wing_angle:.3f} rad",
            f"  FW angle:  {lap.best_design.front_wing_angle:.3f} rad",
            f"  h_f / h_r: {lap.best_design.h_front:.3f} / {lap.best_design.h_rear:.3f} m",
            f"  DRS sched: {lap.best_design.drs_schedule:.2f}",
        ]
    if explore is not None:
        lines += [
            "",
            "AI explorer",
            f"  Candidates: {len(explore.candidates)}",
        ]
        if explore.true_objs is not None and len(explore.true_objs):
            lines.append(
                f"  Best cand DF: {explore.true_objs[:, 0].max():.1f} N"
            )
            lines.append(
                f"  Best cand lap: {explore.true_objs[:, 3].min():.3f} s"
            )
    return "\n".join(lines)
