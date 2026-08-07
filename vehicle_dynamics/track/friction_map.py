"""Spatial friction / surface map along track distance."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .track_segments import SurfaceProperties


class FrictionMap:
    def __init__(self, s_breaks: np.ndarray, mu_values: np.ndarray):
        self.s_breaks = np.asarray(s_breaks, dtype=float)
        self.mu_values = np.asarray(mu_values, dtype=float)

    def mu(self, s: ArrayLike) -> np.ndarray:
        s = np.atleast_1d(np.asarray(s, dtype=float))
        return np.interp(s, self.s_breaks, self.mu_values, left=self.mu_values[0], right=self.mu_values[-1])

    @classmethod
    def from_segments(cls, lengths: list[float], mus: list[float]) -> "FrictionMap":
        s = np.concatenate([[0.0], np.cumsum(lengths)])
        # stepwise: assign mu at segment midpoints then interpolate
        mu_nodes = np.zeros(len(s))
        mu_nodes[0] = mus[0]
        for i, m in enumerate(mus):
            mu_nodes[i + 1] = m
        return cls(s, mu_nodes)

    @classmethod
    def uniform(cls, length: float, mu: float = 1.0) -> "FrictionMap":
        return cls(np.array([0.0, length]), np.array([mu, mu]))
