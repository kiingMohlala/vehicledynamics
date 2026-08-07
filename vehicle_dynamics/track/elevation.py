"""Elevation profile along track distance."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def elevation_at(s: ArrayLike, s_nodes: np.ndarray, z_nodes: np.ndarray) -> np.ndarray:
    s = np.atleast_1d(np.asarray(s, dtype=float))
    return np.interp(s, s_nodes, z_nodes, left=z_nodes[0], right=z_nodes[-1])


def grade_at(s: ArrayLike, s_nodes: np.ndarray, z_nodes: np.ndarray) -> np.ndarray:
    z = elevation_at(s, s_nodes, z_nodes)
    s = np.atleast_1d(np.asarray(s, dtype=float))
    if len(s) < 2:
        return np.zeros_like(s)
    return np.gradient(z, s)
