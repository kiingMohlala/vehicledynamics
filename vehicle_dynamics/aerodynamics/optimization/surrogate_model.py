"""RBF / nearest-neighbor surrogate for aero objectives."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .design_variables import DesignVector


@dataclass
class SurrogateModel:
    X: np.ndarray               # (n, d) normalized
    Y: np.ndarray               # (n, m)
    scale: np.ndarray           # feature scale
    length_scale: float = 1.0

    def predict(self, x: np.ndarray) -> np.ndarray:
        if self.X.shape[0] == 0:
            return np.zeros(self.Y.shape[1] if self.Y.ndim > 1 else 1)
        xn = x / self.scale
        d = np.linalg.norm((self.X - xn) / max(self.length_scale, 1e-6), axis=1)
        if np.min(d) < 1e-12:
            return self.Y[np.argmin(d)].copy()
        w = 1.0 / np.power(np.maximum(d, 1e-12), 2)
        w /= w.sum()
        return w @ self.Y

    def predict_design(self, d: DesignVector) -> np.ndarray:
        return self.predict(d.as_array())


def train_surrogate(
    designs: list[DesignVector],
    objectives: np.ndarray,
    length_scale: float = 1.0,
) -> SurrogateModel:
    X = np.vstack([d.as_array() for d in designs])
    scale = X.std(axis=0) + 1e-9
    Xn = X / scale
    return SurrogateModel(
        X=Xn,
        Y=np.asarray(objectives, dtype=float),
        scale=scale,
        length_scale=length_scale,
    )
