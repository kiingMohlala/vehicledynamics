"""Text reporting for modal analysis."""

from __future__ import annotations

import numpy as np
from .modal_result import ModalResult


def format_modal_report(result: ModalResult, title: str = "Modal Analysis") -> str:
    lines = [
        f"=== {title} ===",
        f"Success: {result.success} ({result.message})",
        f"Mass matrix: {result.mass_type}",
        f"Rigid-body modes (f < tol): {result.n_rigid_body}",
        "",
        f"{'Mode':>4}  {'f [Hz]':>12}  {'T [s]':>10}  {'Classification':<16}",
        "-" * 50,
    ]
    for m in result.modes:
        Tstr = f"{m.period_s:.4f}" if np.isfinite(m.period_s) else "inf"
        lines.append(
            f"{m.index:4d}  {m.frequency_Hz:12.4f}  {Tstr:>10}  {m.classification:<16}"
        )
    return "\n".join(lines)
