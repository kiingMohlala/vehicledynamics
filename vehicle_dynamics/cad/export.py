"""Simple mesh export (OBJ / STL) from AABB box approximations."""
from __future__ import annotations

from pathlib import Path
from typing import Iterable
import numpy as np
from .component import Component


def _box_mesh(c: Component) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    """8 vertices + 12 triangles for AABB."""
    mn, mx = c.aabb_min, c.aabb_max
    v = np.array([
        [mn[0], mn[1], mn[2]],
        [mx[0], mn[1], mn[2]],
        [mx[0], mx[1], mn[2]],
        [mn[0], mx[1], mn[2]],
        [mn[0], mn[1], mx[2]],
        [mx[0], mn[1], mx[2]],
        [mx[0], mx[1], mx[2]],
        [mn[0], mx[1], mx[2]],
    ], dtype=float)
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    return v, faces


def export_obj(components: Iterable[Component], path: str | Path) -> Path:
    path = Path(path)
    verts: list[np.ndarray] = []
    faces: list[str] = []
    offset = 0
    for c in components:
        v, f = _box_mesh(c)
        verts.append(v)
        for a, b, d in f:
            faces.append(f"f {a+offset+1} {b+offset+1} {d+offset+1}")
        offset += len(v)
        faces.append(f"# {c.name}")
    lines = ["# vehicle_dynamics CAD export", f"# components: {offset // 8}"]
    for block in verts:
        for p in block:
            lines.append(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
    lines.extend(faces)
    path.write_text("\n".join(lines) + "\n")
    return path


def export_stl(components: Iterable[Component], path: str | Path) -> Path:
    """ASCII STL."""
    path = Path(path)
    lines = ["solid vehicle"]
    for c in components:
        v, faces = _box_mesh(c)
        for a, b, d in faces:
            n = np.cross(v[b] - v[a], v[d] - v[a])
            nn = np.linalg.norm(n)
            if nn > 1e-15:
                n = n / nn
            else:
                n = np.array([0.0, 0.0, 1.0])
            lines.append(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}")
            lines.append("    outer loop")
            for idx in (a, b, d):
                p = v[idx]
                lines.append(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}")
            lines.append("    endloop")
            lines.append("  endfacet")
    lines.append("endsolid vehicle")
    path.write_text("\n".join(lines) + "\n")
    return path


def export_json_assembly(components: Iterable[Component], path: str | Path, meta: dict | None = None) -> Path:
    import json
    path = Path(path)
    data = {
        "meta": meta or {},
        "components": [c.to_dict() for c in components],
    }
    path.write_text(json.dumps(data, indent=2))
    return path
