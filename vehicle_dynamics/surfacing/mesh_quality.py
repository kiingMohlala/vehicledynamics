"""Mesh quality metrics."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class MeshQuality:
    mean_aspect_ratio: float
    max_aspect_ratio: float
    mean_skewness: float
    max_skewness: float
    normal_consistency: float
    manifold_proxy: bool
    watertight_proxy: bool
    n_faces: int
    n_vertices: int


def triangle_aspect_ratios(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    ar = []
    for f in faces:
        p0, p1, p2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
        e = np.array([np.linalg.norm(p1 - p0), np.linalg.norm(p2 - p1), np.linalg.norm(p0 - p2)])
        if e.min() < 1e-15:
            ar.append(1e6)
        else:
            ar.append(float(e.max() / e.min()))
    return np.asarray(ar, dtype=float)


def triangle_skewness(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """0 = equilateral, →1 bad."""
    sk = []
    for f in faces:
        p0, p1, p2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
        e0, e1, e2 = np.linalg.norm(p1 - p0), np.linalg.norm(p2 - p1), np.linalg.norm(p0 - p2)
        s = 0.5 * (e0 + e1 + e2)
        area = max(s * (s - e0) * (s - e1) * (s - e2), 0.0) ** 0.5
        ideal = (e0 + e1 + e2) / 3.0
        eq_area = (3 ** 0.5 / 4.0) * ideal ** 2
        if eq_area < 1e-15:
            sk.append(1.0)
        else:
            sk.append(float(np.clip(1.0 - area / eq_area, 0.0, 1.0)))
    return np.asarray(sk, dtype=float)


def evaluate_mesh_quality(vertices: np.ndarray, faces: np.ndarray, normals: np.ndarray | None = None) -> MeshQuality:
    if len(faces) == 0:
        return MeshQuality(0, 0, 0, 0, 1.0, True, True, 0, len(vertices))
    ar = triangle_aspect_ratios(vertices, faces)
    sk = triangle_skewness(vertices, faces)
    # normal consistency: adjacent face normal agreement
    consistency = 1.0
    if normals is not None and len(normals) == len(vertices):
        # average |n| after accumulation already unit — check finite
        consistency = float(np.mean(np.isfinite(normals).all(axis=1)))
    # manifold/watertight proxies: every edge shared by 1 or 2 faces; closed if all 2
    from collections import Counter
    edges = []
    for f in faces:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            edges.append(tuple(sorted((int(a), int(b)))))
    cnt = Counter(edges)
    vals = list(cnt.values())
    manifold = all(v <= 2 for v in vals) if vals else True
    watertight = all(v == 2 for v in vals) if vals else True
    # multi-panel open shells are not watertight — that's expected
    return MeshQuality(
        mean_aspect_ratio=float(np.mean(ar)),
        max_aspect_ratio=float(np.max(ar)),
        mean_skewness=float(np.mean(sk)),
        max_skewness=float(np.max(sk)),
        normal_consistency=consistency,
        manifold_proxy=manifold,
        watertight_proxy=watertight,
        n_faces=len(faces),
        n_vertices=len(vertices),
    )
