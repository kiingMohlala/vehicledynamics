"""Markdown / text calibration reports."""
from __future__ import annotations

from pathlib import Path
from typing import Any


def format_calibration_report(result: dict[str, Any], title: str = "Calibration Report") -> str:
    lines = [
        f"# {title}",
        "",
        f"- Method: **{result.get('method', '')}**",
        f"- RMSE: **{result.get('rmse', float('nan')):.6g}**",
        f"- R²: **{result.get('r2', float('nan')):.4f}**",
        f"- Evaluations: **{result.get('nfev', 0)}**",
        f"- Confidence: **{result.get('confidence', float('nan')):.3f}**",
        "",
        "## Best parameters",
        "",
    ]
    for k, v in (result.get("best_parameters") or {}).items():
        lines.append(f"- `{k}` = {v:.6g}")
    if result.get("initial_parameters"):
        lines += ["", "## Parameter changes", ""]
        init = result["initial_parameters"]
        best = result.get("best_parameters") or {}
        for k in best:
            if k in init:
                lines.append(f"- `{k}`: {init[k]:.6g} → {best[k]:.6g}")
    return "\n".join(lines)


def export_calibration_report(result: dict[str, Any], path: str | Path, title: str = "Calibration Report") -> Path:
    path = Path(path)
    path.write_text(format_calibration_report(result, title=title))
    return path
