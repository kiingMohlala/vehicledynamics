"""Mesh quality gates for preprocessing."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.surfacing.mesh_quality import evaluate_mesh_quality, MeshQuality


@dataclass
class MeshValidationReport:
    quality: MeshQuality
    aspect_ok: bool
    skew_ok: bool
    finite_ok: bool
    non_manifold_edges: int


def validate_surface_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
    normals: np.ndarray | None = None,
    max_aspect: float = 100.0,
    max_skew: float = 0.95,
) -> MeshValidationReport:
    q = evaluate_mesh_quality(vertices, faces, normals)
    finite_ok = bool(np.all(np.isfinite(vertices))) and (len(faces) == 0 or np.all(np.isfinite(faces)))
    from .geometry_cleanup import detect_non_manifold_edges
    nm = detect_non_manifold_edges(faces)
    return MeshValidationReport(
        quality=q,
        aspect_ok=q.max_aspect_ratio <= max_aspect,
        skew_ok=q.max_skewness <= max_skew,
        finite_ok=finite_ok,
        non_manifold_edges=len(nm),
    )
