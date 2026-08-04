"""Static linear FEM solver."""

from __future__ import annotations

import numpy as np
from .assembler import Model
from .constraints import free_dofs
from .result import StaticResult


def solve_static(model: Model, loads: np.ndarray) -> StaticResult:
    """
    Solve K u = F with essential BCs eliminated.

    Constrained DOFs are set to zero displacement; reactions recovered as
    R = K u - F at those DOFs.
    """
    K = model.assemble_stiffness()
    F = np.asarray(loads, dtype=float).reshape(-1).copy()
    if F.size != model.ndof:
        raise ValueError(f"Load vector size {F.size} != ndof {model.ndof}")

    free = free_dofs(model)
    fixed = ~free

    if not np.any(free):
        return StaticResult(
            u=np.zeros(model.ndof),
            reactions=-F,
            success=False,
            message="No free DOFs",
        )

    Kff = K[np.ix_(free, free)]
    Ff = F[free]

    # Check conditioning
    try:
        # Symmetry enforcement
        Kff = 0.5 * (Kff + Kff.T)
        u_f = np.linalg.solve(Kff, Ff)
    except np.linalg.LinAlgError as e:
        return StaticResult(
            u=np.zeros(model.ndof),
            reactions=np.zeros(model.ndof),
            success=False,
            message=f"Singular system: {e}",
        )

    if not np.all(np.isfinite(u_f)):
        return StaticResult(
            u=np.zeros(model.ndof),
            reactions=np.zeros(model.ndof),
            success=False,
            message="Non-finite solution",
        )

    u = np.zeros(model.ndof)
    u[free] = u_f

    # Reactions: R = K u - F (nonzero only on fixed DOFs ideally)
    R = K @ u - F

    return StaticResult(u=u, reactions=R, success=True, message="ok")
