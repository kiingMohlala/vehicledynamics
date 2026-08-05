"""Wind gust models: step, ramp, Dryden turbulence."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


class GustModel(ABC):
    @abstractmethod
    def wind(self, t: float) -> np.ndarray:
        """Return wind vector [Wx, Wy, Wz] m/s at time t."""


@dataclass
class StepGust(GustModel):
    """Constant wind after t_onset (default pure crosswind)."""

    magnitude: float = 10.0
    direction_xy: float = np.pi / 2  # rad from +x (π/2 = +y crosswind)
    t_onset: float = 1.0
    t_end: float = np.inf

    def wind(self, t: float) -> np.ndarray:
        if t < self.t_onset or t > self.t_end:
            return np.zeros(3)
        return self.magnitude * np.array([
            np.cos(self.direction_xy),
            np.sin(self.direction_xy),
            0.0,
        ])


@dataclass
class RampGust(GustModel):
    magnitude: float = 10.0
    direction_xy: float = np.pi / 2
    t_onset: float = 1.0
    ramp_time: float = 0.5
    t_end: float = np.inf

    def wind(self, t: float) -> np.ndarray:
        if t < self.t_onset or t > self.t_end:
            return np.zeros(3)
        if t < self.t_onset + self.ramp_time:
            s = (t - self.t_onset) / max(self.ramp_time, 1e-9)
        else:
            s = 1.0
        return s * self.magnitude * np.array([
            np.cos(self.direction_xy),
            np.sin(self.direction_xy),
            0.0,
        ])


@dataclass
class DrydenGust(GustModel):
    """
    Simplified discrete Dryden-like lateral turbulence.

    ẇ = -w/T + σ * sqrt(2/T) * n(t)
    """

    sigma: float = 3.0          # turbulence intensity m/s
    length_scale: float = 150.0  # m
    V_ref: float = 40.0          # m/s for time scale
    seed: int = 0

    def __post_init__(self) -> None:
        self._w = np.zeros(3)
        self._rng = np.random.default_rng(self.seed)
        self._t_prev = 0.0

    def wind(self, t: float) -> np.ndarray:
        dt = max(t - self._t_prev, 0.0)
        self._t_prev = t
        if dt <= 0:
            return self._w.copy()
        T = self.length_scale / max(self.V_ref, 1.0)
        # Lateral + vertical components
        for i in (1, 2):
            n = self._rng.standard_normal()
            self._w[i] += (-self._w[i] / T) * dt + self.sigma * np.sqrt(2.0 * dt / T) * n
        return self._w.copy()

    def reset(self) -> None:
        self._w[:] = 0.0
        self._t_prev = 0.0
