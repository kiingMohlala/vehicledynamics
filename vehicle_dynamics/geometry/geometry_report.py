"""Geometry summary reports."""
from __future__ import annotations

from typing import Any


def format_geometry_report(entity: Any, title: str = "Geometry Report") -> str:
    lines = [f"# {title}", ""]
    name = type(entity).__name__
    lines.append(f"**Type:** {name}")
    if hasattr(entity, "degree"):
        lines.append(f"**Degree:** {entity.degree}")
    if hasattr(entity, "control_points"):
        cp = entity.control_points
        lines.append(f"**Control points:** {len(cp)}")
    if hasattr(entity, "length"):
        try:
            lines.append(f"**Length:** {entity.length():.6g}")
        except TypeError:
            lines.append(f"**Length:** {entity.length:.6g}")
    if hasattr(entity, "n_vertices"):
        lines.append(f"**Vertices:** {entity.n_vertices}")
        lines.append(f"**Faces:** {entity.n_faces}")
    if hasattr(entity, "sample_grid"):
        g = entity.sample_grid(8, 8)
        lines.append(f"**Grid sample:** {g.shape}")
    return "\n".join(lines)
