"""UV parameterization helpers."""
from __future__ import annotations

import numpy as np


def grid_uvs(nu: int, nv: int) -> np.ndarray:
    uu, vv = np.meshgrid(np.linspace(0, 1, nv), np.linspace(0, 1, nu))
    return np.column_stack([uu.ravel(), vv.ravel()])


def normalize_uvs(uvs: np.ndarray) -> np.ndarray:
    uvs = np.asarray(uvs, dtype=float)
    mn = uvs.min(axis=0)
    mx = uvs.max(axis=0)
    span = np.where(mx - mn < 1e-15, 1.0, mx - mn)
    return (uvs - mn) / span
