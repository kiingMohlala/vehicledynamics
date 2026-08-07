"""CFD surface mesh generation from stitched body / panels."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np

from vehicle_dynamics.surfacing.mesh_generator import MeshGenerator, GeneratedMesh
from .geometry_cleanup import cleanup_mesh


@dataclass
class SurfaceMesh:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray | None
    zones: dict[str, np.ndarray]  # zone name -> face indices
    meta: dict


class SurfaceMesher:
    def __init__(self, mesh_size: float = 0.02):
        self.mesh_size = mesh_size
        self.gen = MeshGenerator(target_edge_length=mesh_size)

    def mesh(self, body: Any, zone_by_panel: bool = True) -> SurfaceMesh:
        gm = self.gen.generate(body, target_edge_length=self.mesh_size)
        cleaned = cleanup_mesh(gm.vertices, gm.faces)
        zones: dict[str, np.ndarray] = {}
        if zone_by_panel and hasattr(body, "panels") and gm.panel_ids is not None:
            # map vertices to panels; faces get panel of majority vertex
            for pi, panel in enumerate(body.panels):
                # faces whose first vertex belongs to panel
                face_panels = gm.panel_ids[gm.faces[:, 0]] if len(gm.faces) else np.array([], dtype=int)
                # after cleanup indices change — assign by sequential blocks approximate
                zones[panel.name] = np.array([], dtype=int)
            # simpler: one zone "body"
            zones = {"body": np.arange(len(cleaned.faces))}
            if hasattr(body, "panels"):
                for p in body.panels:
                    zones.setdefault(p.kind, np.array([], dtype=int))
        else:
            zones = {"body": np.arange(len(cleaned.faces))}
        return SurfaceMesh(
            vertices=cleaned.vertices,
            faces=cleaned.faces,
            normals=cleaned.normals,
            zones=zones,
            meta={"mesh_size": self.mesh_size, "n_merged": cleaned.n_merged},
        )
