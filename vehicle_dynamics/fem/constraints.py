"""Boundary conditions and load application."""

from __future__ import annotations

import numpy as np
from .node import Node
from .assembler import Model


def fix_node(node: Node, dofs: list[int] | None = None) -> None:
    """
    Constrain selected DOFs (0..5). Default: all six (clamped).
    """
    if dofs is None:
        dofs = list(range(6))
    for d in dofs:
        node.fixed[d] = True


def pin_node(node: Node) -> None:
    """Fix translations only (ux, uy, uz)."""
    fix_node(node, [0, 1, 2])


def free_dofs(model: Model) -> np.ndarray:
    mask = []
    for n in model.nodes:
        for d in range(6):
            mask.append(not n.fixed[d])
    return np.array(mask, dtype=bool)


def apply_force(F: np.ndarray, node: Node, fx=0.0, fy=0.0, fz=0.0,
                mx=0.0, my=0.0, mz=0.0) -> None:
    idx = node.dof_indices()
    loads = [fx, fy, fz, mx, my, mz]
    for i, val in enumerate(loads):
        F[idx[i]] += val
