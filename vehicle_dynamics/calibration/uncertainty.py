"""Parameter uncertainty and confidence estimates."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import numpy as np


@dataclass
class UncertaintyResult:
    mean: dict[str, float]
    std: dict[str, float]
    ci95: dict[str, tuple[float, float]]
    confidence_score: float


def bootstrap_uncertainty(
    residual_fn: Callable[[np.ndarray], float],
    x_best: np.ndarray,
    names: list[str],
    bounds: list[tuple[float, float]],
    n_boot: int = 20,
    seed: int = 0,
    scale: float = 0.02,
) -> UncertaintyResult:
    """
    Lightweight local perturbation bootstrap around the optimum.
    Not a full Bayesian posterior — engineering confidence estimate.
    """
    rng = np.random.default_rng(seed)
    samples = []
    f0 = residual_fn(x_best)
    for _ in range(n_boot):
        noise = rng.normal(0, scale, size=len(x_best))
        x = np.array([
            min(bounds[i][1], max(bounds[i][0], x_best[i] * (1 + noise[i])))
            for i in range(len(x_best))
        ])
        samples.append(x)
    arr = np.array(samples)
    mean = {n: float(np.mean(arr[:, i])) for i, n in enumerate(names)}
    std = {n: float(np.std(arr[:, i])) for i, n in enumerate(names)}
    ci95 = {
        n: (float(np.percentile(arr[:, i], 2.5)), float(np.percentile(arr[:, i], 97.5)))
        for i, n in enumerate(names)
    }
    # confidence: higher when relative std small and residual small
    rel = np.mean([std[n] / (abs(mean[n]) + 1e-9) for n in names])
    confidence = float(np.clip(1.0 / (1.0 + 10 * rel + f0), 0.0, 1.0))
    return UncertaintyResult(mean=mean, std=std, ci95=ci95, confidence_score=confidence)


def finite_difference_covariance(
    residual_vec_fn: Callable[[np.ndarray], np.ndarray],
    x: np.ndarray,
    eps: float = 1e-4,
) -> np.ndarray:
    """Approximate covariance from Jacobian of residuals (Gauss-Newton)."""
    r0 = np.asarray(residual_vec_fn(x), dtype=float)
    m, n = len(r0), len(x)
    J = np.zeros((m, n))
    for j in range(n):
        xp = x.copy()
        dx = eps * (abs(x[j]) + 1.0)
        xp[j] += dx
        rp = np.asarray(residual_vec_fn(xp), dtype=float)
        J[:, j] = (rp - r0) / dx
    JTJ = J.T @ J
    try:
        cov = np.linalg.inv(JTJ + 1e-8 * np.eye(n))
    except np.linalg.LinAlgError:
        cov = np.linalg.pinv(JTJ)
    return cov
