"""Preprocessing report."""
from __future__ import annotations

from typing import Any


def format_preprocessing_report(result: Any, title: str = "CFD/FEA Preprocessing Report") -> str:
    s = result.surface
    v = result.validation
    lines = [
        f"# {title}",
        "",
        f"**Target:** {result.config.target}",
        f"**Surface vertices:** {len(s.vertices)}",
        f"**Surface faces:** {len(s.faces)}",
        f"**Mesh size:** {result.config.mesh_size}",
        f"**Boundary layers:** {result.config.boundary_layers}",
        "",
        "## Quality",
        f"- Mean aspect ratio: **{v.quality.mean_aspect_ratio:.2f}**",
        f"- Max aspect ratio: **{v.quality.max_aspect_ratio:.2f}** (ok={v.aspect_ok})",
        f"- Mean skewness: **{v.quality.mean_skewness:.3f}** (ok={v.skew_ok})",
        f"- Non-manifold edges: **{v.non_manifold_edges}**",
        "",
        "## Boundary conditions",
    ]
    for bc in result.bcs:
        lines.append(f"- {bc.name}: {bc.bc_type} @ {bc.region}")
    lines += ["", "## Materials"]
    for m in result.materials:
        lines.append(f"- {m.region}: {m.material}")
    if result.volume is not None:
        lines += ["", f"**Volume cells:** {len(result.volume.cells)} ({result.volume.cell_type})"]
    if result.exports:
        lines += ["", "## Exports"]
        for k, p in result.exports.items():
            lines.append(f"- {k}: `{p}`")
    return "\n".join(lines)
