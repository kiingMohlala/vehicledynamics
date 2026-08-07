"""Calibration optimizers: Nelder-Mead, differential evolution, least squares, grid."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Any
import numpy as np


@dataclass
class OptimizeResult:
    x: np.ndarray
    fun: float
    nfev: int
    success: bool
    message: str
    method: str


def nelder_mead(
    fun: Callable[[np.ndarray], float],
    x0: np.ndarray,
    maxiter: int = 200,
    tol: float = 1e-6,
) -> OptimizeResult:
    try:
        from scipy.optimize import minimize
        r = minimize(fun, x0, method="Nelder-Mead", options={"maxiter": maxiter, "xatol": tol, "fatol": tol})
        return OptimizeResult(np.asarray(r.x, dtype=float), float(r.fun), int(r.nfev), bool(r.success), str(r.message), "Nelder-Mead")
    except Exception:
        # simple coordinate descent fallback
        x = np.asarray(x0, dtype=float).copy()
        f = float(fun(x))
        nfev = 1
        step = 0.05 * (np.abs(x) + 1.0)
        for _ in range(maxiter):
            improved = False
            for i in range(len(x)):
                for s in (+step[i], -step[i]):
                    trial = x.copy()
                    trial[i] += s
                    ft = float(fun(trial))
                    nfev += 1
                    if ft < f - tol:
                        x, f = trial, ft
                        improved = True
            step *= 0.8
            if not improved:
                break
        return OptimizeResult(x, f, nfev, True, "coordinate-descent-fallback", "Nelder-Mead")


def differential_evolution(
    fun: Callable[[np.ndarray], float],
    bounds: list[tuple[float, float]],
    maxiter: int = 30,
    seed: int = 0,
    popsize: int = 8,
) -> OptimizeResult:
    try:
        from scipy.optimize import differential_evolution as de
        r = de(fun, bounds, maxiter=maxiter, seed=seed, popsize=popsize, polish=True)
        return OptimizeResult(np.asarray(r.x, dtype=float), float(r.fun), int(r.nfev), bool(r.success), str(r.message), "differential_evolution")
    except Exception:
        rng = np.random.default_rng(seed)
        d = len(bounds)
        lows = np.array([b[0] for b in bounds])
        highs = np.array([b[1] for b in bounds])
        pop = lows + rng.random((max(popsize, 4) * d, d)) * (highs - lows)
        scores = np.array([fun(p) for p in pop])
        nfev = len(pop)
        best_i = int(np.argmin(scores))
        for _ in range(maxiter):
            for i in range(len(pop)):
                a, b, c = pop[rng.choice(len(pop), 3, replace=False)]
                mutant = np.clip(a + 0.7 * (b - c), lows, highs)
                trial = np.where(rng.random(d) < 0.7, mutant, pop[i])
                ft = float(fun(trial))
                nfev += 1
                if ft < scores[i]:
                    pop[i], scores[i] = trial, ft
                    if ft < scores[best_i]:
                        best_i = i
        return OptimizeResult(pop[best_i], float(scores[best_i]), nfev, True, "de-fallback", "differential_evolution")


def least_squares(
    residual_fn: Callable[[np.ndarray], np.ndarray],
    x0: np.ndarray,
    bounds: list[tuple[float, float]] | None = None,
    maxiter: int = 100,
) -> OptimizeResult:
    try:
        from scipy.optimize import least_squares as ls
        kw = {"max_nfev": maxiter * max(1, len(x0))}
        if bounds is not None:
            kw["bounds"] = ([b[0] for b in bounds], [b[1] for b in bounds])
        r = ls(residual_fn, x0, **kw)
        return OptimizeResult(np.asarray(r.x, dtype=float), float(np.sum(r.fun ** 2)), int(r.nfev), bool(r.success), str(r.message), "least_squares")
    except Exception:
        def sse(x):
            r = residual_fn(x)
            return float(np.sum(np.asarray(r, dtype=float) ** 2))
        return nelder_mead(sse, x0, maxiter=maxiter)


def grid_search(
    fun: Callable[[np.ndarray], float],
    bounds: list[tuple[float, float]],
    levels: int = 5,
) -> OptimizeResult:
    levels = max(2, int(levels))
    axes = [np.linspace(b[0], b[1], levels) for b in bounds]
    grids = np.meshgrid(*axes, indexing="ij")
    pts = np.column_stack([g.ravel() for g in grids])
    scores = np.array([fun(p) for p in pts])
    i = int(np.argmin(scores))
    return OptimizeResult(pts[i], float(scores[i]), len(pts), True, "grid", "grid_search")
