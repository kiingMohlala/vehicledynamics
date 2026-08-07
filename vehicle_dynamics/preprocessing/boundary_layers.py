"""Prism / inflation layer generation along surface normals."""
from __future__ import annotations

import numpy as np


def generate_prism_layers(
    vertices: np.ndarray,
    faces: np.ndarray,
    normals: np.ndarray,
    n_layers: int = 5,
    first_height: float = 0.001,
    growth: float = 1.25,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns stacked vertices and wedge cells as (n, 6) vertex indices
    (tri extruded to prism).
    """
    n = len(vertices)
    heights = [first_height * (growth ** i) for i in range(n_layers)]
    offsets = np.cumsum(heights)
    all_verts = [vertices]
    for h in offsets:
        all_verts.append(vertices + normals * h)
    verts = np.vstack(all_verts)
    cells = []
    for layer in range(n_layers):
        base = layer * n
        top = (layer + 1) * n
        for f in faces:
            cells.append([
                base + f[0], base + f[1], base + f[2],
                top + f[0], top + f[1], top + f[2],
            ])
    return verts, np.array(cells, dtype=int)
