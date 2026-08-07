"""Surface tessellation into triangle meshes."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Tessellation:
    vertices: np.ndarray   # (V, 3)
    faces: np.ndarray      # (F, 3) int indices
    normals: np.ndarray | None = None

    @property
    def n_vertices(self) -> int:
        return len(self.vertices)

    @property
    def n_faces(self) -> int:
        return len(self.faces)


def tessellate_grid(grid: np.ndarray) -> Tessellation:
    """Convert (nu, nv, 3) grid into triangle mesh."""
    nu, nv, _ = grid.shape
    verts = grid.reshape(-1, 3)
    faces = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            i00 = i * nv + j
            i10 = (i + 1) * nv + j
            i01 = i * nv + j + 1
            i11 = (i + 1) * nv + j + 1
            faces.append([i00, i10, i11])
            faces.append([i00, i11, i01])
    faces = np.array(faces, dtype=int)
    # face normals
    normals = np.zeros_like(verts)
    for f in faces:
        n = np.cross(verts[f[1]] - verts[f[0]], verts[f[2]] - verts[f[0]])
        nn = np.linalg.norm(n)
        if nn > 1e-15:
            n /= nn
        for idx in f:
            normals[idx] += n
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, norms, out=np.zeros_like(normals), where=norms > 1e-15)
    return Tessellation(vertices=verts, faces=faces, normals=normals)


def tessellate_surface(surface, nu: int = 20, nv: int = 20) -> Tessellation:
    grid = surface.sample_grid(nu, nv)
    return tessellate_grid(grid)
