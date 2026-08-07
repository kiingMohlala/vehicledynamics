"""Telemetry CSV / JSON / Markdown export for lap results."""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


def export_csv(path: str | Path, columns: dict[str, list]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(columns.keys())
    n = max((len(v) for v in columns.values()), default=0)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for i in range(n):
            row = []
            for k in keys:
                col = columns[k]
                row.append(col[i] if i < len(col) else "")
            w.writerow(row)


def export_json(path: str | Path, payload: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2, default=float)


def export_markdown_report(path: str | Path, title: str, body: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"# {title}\n\n{body}\n")
