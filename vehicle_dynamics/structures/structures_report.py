"""Structural analysis report."""
from __future__ import annotations

from typing import Any


def format_structures_report(result: Any, title: str = "Structural Analysis Report") -> str:
    lines = [
        f"# {title}",
        "",
        f"**Load case:** {result.load_case}",
        f"**Solver success:** {result.solution.success}",
        "",
        "## Stiffness",
        f"- Torsional: **{result.torsional_stiffness:.1f} Nm/deg**",
        f"- Bending: **{result.bending_stiffness:.1f} N/m**",
        "",
        "## Response",
        f"- Max displacement: **{result.max_displacement*1000:.3f} mm**",
        f"- Max von Mises (proxy): **{result.max_von_mises/1e6:.1f} MPa**",
        "",
        "## Safety",
        f"- Yield SF: **{result.safety.yield_sf:.2f}**",
        f"- Buckling SF: **{result.safety.buckling_sf:.2f}**",
        f"- Fatigue SF: **{result.safety.fatigue_sf:.2f}**",
        f"- Governing: **{result.safety.governing}** ({'OK' if result.safety.ok else 'FAIL'})",
    ]
    if result.modal and result.modal.success and len(result.modal.frequencies_hz):
        lines += ["", "## Modal"]
        for i, f in enumerate(result.modal.frequencies_hz[:5], 1):
            lines.append(f"- Mode {i}: **{f:.2f} Hz**")
    if result.reactions_summary:
        lines += ["", "## Reactions"]
        for k, v in result.reactions_summary.items():
            lines.append(f"- {k}: **{v:.1f} N**")
    return "\n".join(lines)
