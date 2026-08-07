"""Compliance / program status reports."""
from __future__ import annotations

from typing import Any


def format_compliance_report(data: dict[str, Any], title: str = "Engineering Compliance Report") -> str:
    lines = [
        f"# {title}",
        "",
        f"**Program:** {data.get('program', '')}",
        f"**Revision:** {data.get('revision', '')}",
        f"**Baseline:** {data.get('baseline', '')}",
        "",
        "## Requirements",
        "",
        "```",
        data.get("verification_table", ""),
        "```",
        "",
        "## Sign-off",
        "",
    ]
    for stage, status in (data.get("signoff") or {}).items():
        lines.append(f"- {stage}: **{status}**")
    lines += [
        "",
        f"**Release ready:** {data.get('release_ready', False)}",
        f"**Evidence items:** {data.get('n_evidence', 0)}",
        f"**Revisions:** {data.get('n_revisions', 0)}",
    ]
    return "\n".join(lines)
