"""Bundle vehicle, track, telemetry, and reports into a reproducible project package."""
from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import shutil


def export_project(
    root: str | Path,
    *,
    vehicle: dict[str, Any] | None = None,
    track: dict[str, Any] | None = None,
    telemetry: dict[str, Any] | None = None,
    results: dict[str, Any] | None = None,
    reports: dict[str, str] | None = None,
    meta: dict[str, Any] | None = None,
) -> Path:
    root = Path(root)
    if root.exists():
        shutil.rmtree(root)
    for sub in ("vehicle", "track", "telemetry", "results", "reports"):
        (root / sub).mkdir(parents=True)

    if vehicle is not None:
        (root / "vehicle" / "definition.json").write_text(json.dumps(vehicle, indent=2, default=float))
    if track is not None:
        (root / "track" / "track.json").write_text(json.dumps(track, indent=2, default=float))
    if telemetry is not None:
        (root / "telemetry" / "log.json").write_text(json.dumps(telemetry, indent=2, default=float))
    if results is not None:
        (root / "results" / "summary.json").write_text(json.dumps(results, indent=2, default=float))
    if reports:
        for name, body in reports.items():
            (root / "reports" / name).write_text(body)

    manifest = {
        "format": "vehicle_dynamics_project_v1",
        "meta": meta or {},
        "contents": {
            "vehicle": vehicle is not None,
            "track": track is not None,
            "telemetry": telemetry is not None,
            "results": results is not None,
            "reports": list(reports.keys()) if reports else [],
        },
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return root


def load_project(root: str | Path) -> dict[str, Any]:
    root = Path(root)
    manifest = json.loads((root / "manifest.json").read_text())
    out: dict[str, Any] = {"manifest": manifest}
    vp = root / "vehicle" / "definition.json"
    if vp.exists():
        out["vehicle"] = json.loads(vp.read_text())
    tp = root / "track" / "track.json"
    if tp.exists():
        out["track"] = json.loads(tp.read_text())
    lp = root / "telemetry" / "log.json"
    if lp.exists():
        out["telemetry"] = json.loads(lp.read_text())
    rp = root / "results" / "summary.json"
    if rp.exists():
        out["results"] = json.loads(rp.read_text())
    return out
