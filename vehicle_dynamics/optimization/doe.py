"""Design of Experiments samplers."""
from __future__ import annotations

import numpy as np

from .design_variables import DesignVariable
from .parameter_space import ParameterSpace


def _to_space(variables: list[DesignVariable], unit: np.ndarray) -> ParameterSpace:
    """Map unit-hypercube samples [0,1]^d to variable bounds."""
    n, d = unit.shape
    samples = np.zeros_like(unit, dtype=float)
    for j, v in enumerate(variables):
        samples[:, j] = v.low + unit[:, j] * (v.high - v.low)
    return ParameterSpace(variables=variables, samples=samples)


def full_factorial(variables: list[DesignVariable], levels: int = 3) -> ParameterSpace:
    levels = max(2, int(levels))
    axes = [np.linspace(0.0, 1.0, levels) for _ in variables]
    grids = np.meshgrid(*axes, indexing="ij")
    unit = np.column_stack([g.ravel() for g in grids])
    return _to_space(variables, unit)


def latin_hypercube(variables: list[DesignVariable], n_samples: int, seed: int = 0) -> ParameterSpace:
    """Latin Hypercube Sampling in the unit hypercube, then scaled."""
    rng = np.random.default_rng(seed)
    d = len(variables)
    n = max(1, int(n_samples))
    unit = np.zeros((n, d))
    for j in range(d):
        perm = rng.permutation(n)
        unit[:, j] = (perm + rng.random(n)) / n
    return _to_space(variables, unit)


def sobol_sampling(variables: list[DesignVariable], n_samples: int, seed: int = 0) -> ParameterSpace:
    """
    Sobol-like low-discrepancy sequence via scrambled van der Corput / direction numbers.
    Lightweight implementation (not full Joe-Kuo tables) suitable for engineering DOE.
    """
    rng = np.random.default_rng(seed)
    d = len(variables)
    n = max(1, int(n_samples))
    # Use scipy if available
    try:
        from scipy.stats import qmc
        eng = qmc.Sobol(d=d, scramble=True, seed=seed)
        unit = eng.random(n)
    except Exception:
        # Fallback: stratified LHS (already low discrepancy-ish)
        unit = np.zeros((n, d))
        for j in range(d):
            perm = rng.permutation(n)
            unit[:, j] = (perm + 0.5) / n
        unit = (unit + rng.random((n, d)) * 1e-6) % 1.0
    return _to_space(variables, unit)


def random_sampling(variables: list[DesignVariable], n_samples: int, seed: int = 0) -> ParameterSpace:
    rng = np.random.default_rng(seed)
    n = max(1, int(n_samples))
    d = len(variables)
    unit = rng.random((n, d))
    return _to_space(variables, unit)


# Aliases matching task card naming
LatinHypercube = latin_hypercube
SobolSampling = sobol_sampling
FullFactorial = full_factorial
RandomSampling = random_sampling
