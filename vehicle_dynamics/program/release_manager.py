"""Generate release packages with frozen artifacts."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import json
import time
import shutil


@dataclass
class ReleasePackage:
    release_id: str
    program_name: str
    version: str
    path: Path
    contents: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass
class ReleaseManager:
    def create(
        self,
        root: str | Path,
        release_id: str,
        program_name: str,
        version: str,
        *,
        vehicle: dict[str, Any] | None = None,
        verification_matrix: dict[str, Any] | None = None,
        calibration_summary: dict[str, Any] | None = None,
        optimization_summary: dict[str, Any] | None = None,
        requirements: list[dict[str, Any]] | None = None,
        software_version: str = "",
        git_tag: str = "",
        reports: dict[str, str] | None = None,
    ) -> ReleasePackage:
        root = Path(root)
        if root.exists():
            shutil.rmtree(root)
        for sub in ("vehicle", "requirements", "verification", "calibration", "optimization", "reports"):
            (root / sub).mkdir(parents=True)

        contents = []
        if vehicle is not None:
            (root / "vehicle" / "definition.json").write_text(json.dumps(vehicle, indent=2, default=float))
            contents.append("vehicle/definition.json")
        if requirements is not None:
            (root / "requirements" / "requirements.json").write_text(json.dumps(requirements, indent=2))
            contents.append("requirements/requirements.json")
        if verification_matrix is not None:
            (root / "verification" / "matrix.json").write_text(json.dumps(verification_matrix, indent=2, default=float))
            contents.append("verification/matrix.json")
        if calibration_summary is not None:
            (root / "calibration" / "summary.json").write_text(json.dumps(calibration_summary, indent=2, default=float))
            contents.append("calibration/summary.json")
        if optimization_summary is not None:
            (root / "optimization" / "summary.json").write_text(json.dumps(optimization_summary, indent=2, default=float))
            contents.append("optimization/summary.json")
        if reports:
            for name, body in reports.items():
                (root / "reports" / name).write_text(body)
                contents.append(f"reports/{name}")

        manifest = {
            "release_id": release_id,
            "program": program_name,
            "version": version,
            "software_version": software_version,
            "git_tag": git_tag,
            "contents": contents,
            "timestamp": time.time(),
        }
        (root / "manifest.json").write_text(json.dumps(manifest, indent=2))
        contents.append("manifest.json")
        return ReleasePackage(
            release_id=release_id,
            program_name=program_name,
            version=version,
            path=root,
            contents=contents,
        )
