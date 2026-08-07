"""Markdown / text engineering reports for DOE campaigns."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from .batch_runner import BatchResult
from .results_analysis import analyze, summarize


def format_report(result: BatchResult, title: str = "DOE Campaign Report") -> str:
    s = summarize(result)
    analysis = analyze(result)
    sens = analysis["sensitivity"]
    pf = analysis["pareto"]
    lines = [
        f"# {title}",
        "",
        f"- Samples: **{s['n']}**",
        f"- Feasible: **{s['n_feasible']}**",
        f"- Best objective: **{s['best_objective']:.6g}**",
        f"- Mean ± std: **{s['mean_objective']:.6g} ± {s['std_objective']:.6g}**",
        "",
        "## Best design",
        "",
    ]
    for k, v in s["best_design"].items():
        lines.append(f"- `{k}` = {v:.6g}")
    lines += ["", "## Sensitivity ranking", ""]
    for name, score in sens.rankings[:10]:
        lines.append(f"- {name}: {score:.4f}")
    lines += ["", f"## Pareto front ({pf.size} points)", ""]
    for i, pt in enumerate(pf.points[:10]):
        lines.append(f"- P{i}: " + ", ".join(f"{k}={v:.4g}" for k, v in pt.items()))
    return "\n".join(lines)


def export_report(result: BatchResult, path: str | Path, title: str = "DOE Campaign Report") -> Path:
    path = Path(path)
    path.write_text(format_report(result, title=title))
    return path
