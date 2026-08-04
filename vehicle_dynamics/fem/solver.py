"""Static linear FEM solver with under-constrained detection."""

from __future__ import annotations

import numpy as np
from .assembler import Model
from .constraints import free_dofs
from .result import StaticResult


def solve_static(model: Model, loads: np.ndarray) -> StaticResult:
    K = model.assemble_stiffness()
    F = np.asarray(loads, dtype=float).reshape(-1).copy()
    if F.size != model.ndof:
        raise ValueError(f"Load vector size {F.size} != ndof {model.ndof}")

    free = free_dofs(model)

    if not np.any(free):
        return StaticResult(
            u=np.zeros(model.ndof),
            reactions=-F,
            success=False,
            message="No free DOFs",
        )

    n_fixed = int(np.sum(~free))
    if n_fixed < 6:
        return StaticResult(
            u=np.zeros(model.ndof),
            reactions=np.zeros(model.ndof),
            success=False,
            message=f"Under-constrained model ({n_fixed} fixed DOFs < 6)",
        )

    Kff = K[np.ix_(free, free)]
    Ff = F[free]
    Kff = 0.5 * (Kff + Kff.T)

    try:
        cond = np.linalg.cond(Kff)
        if not np.isfinite(cond) or cond > 1e14:
            return StaticResult(
                u=np.zeros(model.ndof),
                reactions=np.zeros(model.ndof),
                success=False,
                message=f"Ill-conditioned / singular system (cond={cond:.2e})",
            )
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
    R = K @ u - F
    max_disp = float(np.max(np.abs(u[0::6]**2 + u[1::6]**2 + u[2::6]**2) ** 0.5))

    return StaticResult(
        u=u,
        reactions=R,
        success=True,
        message="ok",
        max_displacement=max_disp,
    )
