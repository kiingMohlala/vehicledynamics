"""Text / structured verification reports."""

from __future__ import annotations

from pathlib import Path
from datetime import datetime, timezone


def format_verification_report(payload: dict) -> str:
    lines = [
        "=== Vehicle Dynamics Verification Report ===",
        f"Generated: {datetime.now(timezone.utc).isoformat()}",
        "",
    ]
    if "baselines_captured" in payload:
        lines.append(f"Baselines captured: {payload['baselines_captured']}")
    if "baseline_checks" in payload:
        lines.append("\n-- Baseline regression --")
        for c in payload["baseline_checks"]:
            lines.append(f"  {c.name}: {'PASS' if c.ok else 'FAIL'}")
    if "proving_ground" in payload:
        lines.append("\n-- Proving ground --")
        for p in payload["proving_ground"]:
            lines.append(f"  [{p.category}] {p.name}: {'PASS' if p.ok else 'FAIL'}")
    if "matrix" in payload:
        lines.append("\n-- Scenario matrix --")
        for name, ok, det in payload["matrix"]:
            lines.append(f"  {name}: {'PASS' if ok else 'FAIL'}")
    if "benchmarks" in payload:
        lines.append("\n-- Performance --")
        for b in payload["benchmarks"]:
            lines.append(
                f"  {b.name}: {b.ms_per_step:.3f} ms/step  ({b.steps_per_s:.0f} steps/s) "
                f"{'PASS' if b.ok else 'FAIL'}"
            )
    lines.append("")
    lines.append(
        "OVERALL: PASS" if payload.get("all_pass") else "OVERALL: FAIL"
    )
    return "\n".join(lines)


def write_text_report(payload: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(format_verification_report(payload))
    return path
