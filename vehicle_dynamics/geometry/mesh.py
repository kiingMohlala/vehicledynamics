"""Mesh utilities and generation helpers."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .tessellation import Tessellation, tessellate_surface


@dataclass
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray | None = None
    name: str = ""

    @classmethod
    def from_tessellation(cls, tess: Tessellation, name: str = "") -> "Mesh":
        return cls(vertices=tess.vertices, faces=tess.faces, normals=tess.normals, name=name)

    @classmethod
    def from_surface(cls, surface, nu: int = 20, nv: int = 20, name: str = "") -> "Mesh":
        return cls.from_tessellation(tessellate_surface(surface, nu, nv), name=name)

    @property
    def n_vertices(self) -> int:
        return len(self.vertices)

    @property
    def n_faces(self) -> int:
        return len(self.faces)

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)
