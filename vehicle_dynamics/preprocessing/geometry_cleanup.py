"""Mesh geometry cleanup: merge vertices, fix normals, drop degenerates."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class CleanupResult:
    vertices: np.ndarray
    faces: np.ndarray
    n_merged: int
    n_degenerate_removed: int
    normals: np.ndarray | None = None


def merge_duplicate_vertices(
    vertices: np.ndarray,
    faces: np.ndarray,
    tol: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, int]:
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if len(vertices) == 0:
        return vertices, faces, 0
    # quantize
    q = np.round(vertices / tol).astype(np.int64)
    # unique rows
    _, inv, counts = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    new_verts = np.zeros((len(counts), 3))
    for i in range(len(vertices)):
        new_verts[inv[i]] += vertices[i]
    new_verts /= counts[:, None]
    new_faces = inv[faces]
    n_merged = int(len(vertices) - len(new_verts))
    return new_verts, new_faces, n_merged


def remove_degenerate_faces(vertices: np.ndarray, faces: np.ndarray, area_tol: float = 1e-14) -> tuple[np.ndarray, int]:
    keep = []
    for f in faces:
        if len(set(f.tolist())) < 3:
            continue
        p0, p1, p2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
        area = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0))
        if area > area_tol:
            keep.append(f)
    removed = len(faces) - len(keep)
    return (np.array(keep, dtype=int) if keep else np.zeros((0, 3), dtype=int)), removed


def recompute_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(vertices)
    for f in faces:
        n = np.cross(vertices[f[1]] - vertices[f[0]], vertices[f[2]] - vertices[f[0]])
        nn = np.linalg.norm(n)
        if nn > 1e-15:
            n /= nn
        for idx in f:
            normals[idx] += n
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.divide(normals, norms, out=np.zeros_like(normals), where=norms > 1e-15)


def cleanup_mesh(vertices: np.ndarray, faces: np.ndarray, tol: float = 1e-6) -> CleanupResult:
    v, f, n_merged = merge_duplicate_vertices(vertices, faces, tol=tol)
    f2, n_deg = remove_degenerate_faces(v, f)
    normals = recompute_normals(v, f2) if len(f2) else None
    return CleanupResult(v, f2, n_merged, n_deg, normals)


def detect_non_manifold_edges(faces: np.ndarray) -> list[tuple[int, int]]:
    from collections import Counter
    edges = []
    for f in faces:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            edges.append(tuple(sorted((int(a), int(b)))))
    cnt = Counter(edges)
    return [e for e, c in cnt.items() if c > 2]
