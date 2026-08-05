"""Pareto dominance utilities."""

from __future__ import annotations

import numpy as np


def dominates(a: np.ndarray, b: np.ndarray, maximize_mask: np.ndarray) -> bool:
    """
    a dominates b if a is no worse in all objs and better in at least one.
    maximize_mask[i]=True means objective i is maximized.
    """
    a_cmp = np.where(maximize_mask, a, -a)
    b_cmp = np.where(maximize_mask, b, -b)
    return bool(np.all(a_cmp >= b_cmp - 1e-12) and np.any(a_cmp > b_cmp + 1e-12))


def pareto_front(
    objs: np.ndarray,
    maximize_mask: np.ndarray | None = None,
) -> np.ndarray:
    """
    objs: (n, m) objective matrix.
    Returns indices of non-dominated points.
    Default: maximize col0 (DF), minimize col1 (drag).
    """
    n, m = objs.shape
    if maximize_mask is None:
        maximize_mask = np.array([True] + [False] * (m - 1))
    keep = []
    for i in range(n):
        dominated = False
        for j in range(n):
            if i == j:
                continue
            if dominates(objs[j], objs[i], maximize_mask):
                dominated = True
                break
        if not dominated:
            keep.append(i)
    return np.asarray(keep, dtype=int)
