"""Surfacing summary report."""
from __future__ import annotations

from typing import Any


def format_surfacing_report(body_or_mesh: Any, quality: Any = None, title: str = "Surfacing Report") -> str:
    lines = [f"# {title}", ""]
    if hasattr(body_or_mesh, "panels"):
        lines.append(f"**Panels:** {len(body_or_mesh.panels)}")
        for p in body_or_mesh.panels:
            lines.append(f"- {p.name} ({p.kind})")
    if hasattr(body_or_mesh, "n_vertices"):
        lines += ["", f"**Vertices:** {body_or_mesh.n_vertices}", f"**Faces:** {body_or_mesh.n_faces}"]
    if quality is not None:
        lines += [
            "",
            "## Mesh quality",
            f"- Mean aspect ratio: **{quality.mean_aspect_ratio:.2f}**",
            f"- Max aspect ratio: **{quality.max_aspect_ratio:.2f}**",
            f"- Mean skewness: **{quality.mean_skewness:.3f}**",
            f"- Manifold proxy: **{quality.manifold_proxy}**",
            f"- Watertight proxy: **{quality.watertight_proxy}**",
        ]
    return "\n".join(lines)
