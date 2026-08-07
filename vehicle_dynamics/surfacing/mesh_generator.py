"""Adaptive triangulation from panel surfaces."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Any
import numpy as np

from vehicle_dynamics.geometry.tessellation import tessellate_grid, Tessellation
from vehicle_dynamics.geometry.mesh import Mesh


@dataclass
class GeneratedMesh:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray | None = None
    uvs: np.ndarray | None = None
    panel_ids: np.ndarray | None = None

    @property
    def n_vertices(self) -> int:
        return len(self.vertices)

    @property
    def n_faces(self) -> int:
        return len(self.faces)

    def to_mesh(self, name: str = "") -> Mesh:
        return Mesh(vertices=self.vertices, faces=self.faces, normals=self.normals, name=name)


class MeshGenerator:
    def __init__(self, target_edge_length: float = 0.05, min_nu: int = 8, max_nu: int = 60):
        self.target_edge_length = target_edge_length
        self.min_nu = min_nu
        self.max_nu = max_nu

    def _resolution_for_panel(self, panel) -> tuple[int, int]:
        grid = panel.sample_grid(12, 12)
        # approximate extent
        extent = float(np.linalg.norm(grid.max(axis=(0, 1)) - grid.min(axis=(0, 1))))
        n = int(np.clip(extent / max(self.target_edge_length, 1e-4), self.min_nu, self.max_nu))
        return n, max(self.min_nu, n // 2)

    def generate_panel(self, panel, nu: int | None = None, nv: int | None = None) -> GeneratedMesh:
        if nu is None or nv is None:
            nu0, nv0 = self._resolution_for_panel(panel)
            nu = nu or nu0
            nv = nv or nv0
        grid = panel.sample_grid(nu, nv)
        tess = tessellate_grid(grid)
        # UV from grid params
        uu, vv = np.meshgrid(np.linspace(0, 1, nv), np.linspace(0, 1, nu))
        uvs = np.column_stack([uu.ravel(), vv.ravel()])
        return GeneratedMesh(
            vertices=tess.vertices,
            faces=tess.faces,
            normals=tess.normals,
            uvs=uvs,
        )

    def generate(self, body_or_panels, target_edge_length: float | None = None) -> GeneratedMesh:
        if target_edge_length is not None:
            self.target_edge_length = target_edge_length
        if hasattr(body_or_panels, "panels"):
            panels = body_or_panels.panels
        else:
            panels = list(body_or_panels)
        all_v = []
        all_f = []
        all_n = []
        all_uv = []
        all_id = []
        offset = 0
        for pi, panel in enumerate(panels):
            gm = self.generate_panel(panel)
            all_v.append(gm.vertices)
            all_f.append(gm.faces + offset)
            if gm.normals is not None:
                all_n.append(gm.normals)
            if gm.uvs is not None:
                all_uv.append(gm.uvs)
            all_id.append(np.full(len(gm.vertices), pi, dtype=int))
            offset += len(gm.vertices)
        verts = np.vstack(all_v) if all_v else np.zeros((0, 3))
        faces = np.vstack(all_f) if all_f else np.zeros((0, 3), dtype=int)
        normals = np.vstack(all_n) if all_n else None
        uvs = np.vstack(all_uv) if all_uv else None
        pids = np.concatenate(all_id) if all_id else None
        return GeneratedMesh(verts, faces, normals, uvs, pids)
