"""Incremental load stepping for nonlinear statics."""

from __future__ import annotations

import numpy as np
from vehicle_dynamics.fem.assembler import Model

from .nonlinear_solver import solve_static_nonlinear, NonlinearResult
from .convergence import ConvergenceLog


def solve_nonlinear_stepped(
    model: Model,
    loads: np.ndarray,
    n_steps: int = 10,
    tol: float = 1e-6,
    max_iter: int = 30,
) -> NonlinearResult:
    """
    Apply load in equal increments λ = 1/n, 2/n, …, 1.
    Reuses the previous solution as the next initial guess.
    """
    if n_steps < 1:
        raise ValueError("n_steps must be >= 1")

    u = np.zeros(model.ndof)
    combined_log = ConvergenceLog()
    last: NonlinearResult | None = None

    for s in range(1, n_steps + 1):
        lam = s / n_steps
        res = solve_static_nonlinear(
            model,
            loads,
            tol=tol,
            max_iter=max_iter,
            load_factor=lam,
            u0=u,
        )
        for rec in res.log.records:
            combined_log.add(
                rec.iteration,
                rec.residual_norm,
                rec.correction_norm,
                load_factor=lam,
                converged=rec.converged,
            )
        if not res.success:
            res.log = combined_log
            res.message = f"Failed at load factor {lam:.3f}: {res.message}"
            return res
        u = res.u.copy()
        last = res

    assert last is not None
    last.log = combined_log
    last.load_factor = 1.0
    last.message = f"ok ({n_steps} steps)"
    return last
