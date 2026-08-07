"""Summary report for I/O operations."""
from __future__ import annotations

from typing import Any


def format_io_report(info: dict[str, Any]) -> str:
    lines = ["I/O Interface Report", "=" * 40]
    for k, v in info.items():
        lines.append(f"  {k:24s}: {v}")
    return "\n".join(lines)
