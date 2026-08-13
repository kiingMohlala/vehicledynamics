"""Manufacturing report formatter."""
from __future__ import annotations

from typing import Any


def format_manufacturing_report(result: Any, title: str = "Manufacturing Engineering Report") -> str:
    c = result.cost
    lines = [
        f"# {title}",
        "",
        f"**Parts:** {len(result.bom.items)}",
        f"**Manufacturability score:** {result.manufacturability_score:.1f}/100",
        f"**Assembly time:** {result.assembly_time_hours:.2f} h",
        f"**Total cost:** ${result.total_cost:,.2f}",
        "",
        "## Cost breakdown",
        f"- Material: ${c.material:,.2f}",
        f"- Machining: ${c.machining:,.2f}",
        f"- Welding: ${c.welding:,.2f}",
        f"- Composite: ${c.composite:,.2f}",
        f"- Additive: ${c.additive:,.2f}",
        f"- Assembly: ${c.assembly:,.2f}",
        f"- Overhead: ${c.overhead:,.2f}",
        "",
        "## DFM",
        f"- Score: {result.dfm.score:.1f}",
        f"- Issues: {len(result.dfm.issues)}",
        "",
        "## DFA",
        f"- Part count: {result.dfa.part_count}",
        f"- Fasteners: {result.dfa.fastener_count}",
        f"- Score: {result.dfa.score:.1f}",
        "",
        "## Assembly sequence (first 8)",
    ]
    for s in result.assembly_plan.steps[:8]:
        lines.append(f"{s.order}. {s.action} **{s.part}** ({s.time_min:.0f} min)")
    lines += ["", "## BOM (top items)"]
    for item in result.bom.items[:10]:
        lines.append(f"- {item.part_number} {item.name}: {item.material}/{item.process} ${item.unit_cost:.2f}")
    return "\n".join(lines)
