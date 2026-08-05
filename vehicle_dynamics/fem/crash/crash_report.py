"""Crash analysis report."""

from __future__ import annotations

from .crash_solver import CrashResult
from .plastic_hinge import HingeState


def format_crash_report(result: CrashResult, title: str = "Crash Report") -> str:
    e = result.energy
    i = result.intrusion
    lines = [
        f"=== {title} ===",
        f"Success: {result.success} ({result.message})",
        f"Load factor reached: {result.load_factor:.2f}",
        "",
        "Energy",
        f"  Impact KE:        {e.kinetic_initial/1e3:.2f} kJ",
        f"  Elastic strain:   {e.elastic_strain/1e3:.2f} kJ",
        f"  Plastic work:     {e.plastic_work/1e3:.2f} kJ",
        f"  Absorbed:         {e.absorbed/1e3:.2f} kJ",
        f"  Residual KE:      {e.residual_kinetic/1e3:.2f} kJ",
        f"  Crush distance:   {e.crush_distance*1e3:.1f} mm",
        f"  Balance error:    {e.balance_error*100:.1f} %",
        "",
        "Hinges",
        f"  Yielding: {result.n_yielded}  Plastic: {result.n_plastic}  Failed: {result.n_failed}",
        "",
        "Intrusion",
        f"  Max node:      {i.max_node_disp_m*1e3:.1f} mm",
        f"  Seat:          {i.seat_disp_m*1e3:.1f} mm",
        f"  Harness:       {i.harness_disp_m*1e3:.1f} mm",
        f"  Survival cell: {i.survival_cell_intrusion_m*1e3:.1f} mm",
        f"  Peak decel:    {i.peak_decel_g:.1f} g",
    ]
    # Top degraded members
    ranked = sorted(
        result.hinge_states.values(),
        key=lambda s: -max(s.M_ratio, s.N_ratio),
    )
    lines.append("\nCritical members:")
    for s in ranked[:8]:
        lines.append(
            f"  id={s.elem_id:3d}  {s.state.value:8s}  "
            f"M_ratio={s.M_ratio:.2f}  deg={s.degradation:.2f}"
        )
    return "\n".join(lines)
