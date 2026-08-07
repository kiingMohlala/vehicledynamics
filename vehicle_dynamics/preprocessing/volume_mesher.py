"""Simplified volume mesh: extrude surface along normals (prism) or tetrahedralize AABB fill proxy."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .boundary_layers import generate_prism_layers


@dataclass
class VolumeMesh:
    vertices: np.ndarray
    cells: np.ndarray  # (C, 4) tets or (C, 6) wedges encoded as -1 padded
    cell_type: str  # "tet" | "prism"
    zones: dict


class VolumeMesher:
    def __init__(self, domain_scale: float = 5.0):
        self.domain_scale = domain_scale

    def mesh_from_surface(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        normals: np.ndarray | None = None,
        n_layers: int = 3,
        first_height: float = 0.002,
        growth: float = 1.2,
    ) -> VolumeMesh:
        """Generate prism boundary-layer stack; far-field left as domain AABB corners (diagnostic)."""
        if normals is None:
            normals = np.zeros_like(vertices)
            normals[:, 2] = 1.0
        verts, cells = generate_prism_layers(
            vertices, faces, normals,
            n_layers=n_layers,
            first_height=first_height,
            growth=growth,
        )
        return VolumeMesh(vertices=verts, cells=cells, cell_type="prism", zones={"fluid": np.arange(len(cells))})

    def domain_box(self, vertices: np.ndarray) -> np.ndarray:
        mn, mx = vertices.min(axis=0), vertices.max(axis=0)
        c = 0.5 * (mn + mx)
        half = 0.5 * (mx - mn) * self.domain_scale
        return np.array([c - half, c + half])
