"""Simple surrogate models: polynomial response surface and IDW."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass
class SurrogateModel:
    kind: str
    X: np.ndarray
    y: np.ndarray
    coef: np.ndarray | None = None

    def predict(self, x: np.ndarray) -> float:
        x = np.asarray(x, dtype=float).ravel()
        if self.kind == "poly2" and self.coef is not None:
            # [1, x..., x_i*x_j...]
            feats = _poly2_features(x.reshape(1, -1))[0]
            return float(feats @ self.coef)
        if self.kind == "idw":
            return float(_idw_predict(self.X, self.y, x))
        # nearest
        d = np.linalg.norm(self.X - x, axis=1)
        return float(self.y[int(np.argmin(d))])


def _poly2_features(X: np.ndarray) -> np.ndarray:
    n, d = X.shape
    cols = [np.ones(n)]
    for j in range(d):
        cols.append(X[:, j])
    for i in range(d):
        for j in range(i, d):
            cols.append(X[:, i] * X[:, j])
    return np.column_stack(cols)


def fit_polynomial(X: np.ndarray, y: np.ndarray) -> SurrogateModel:
    X = np.asarray(X, dtype=float)
    y = np.asarray(y, dtype=float).ravel()
    F = _poly2_features(X)
    coef, *_ = np.linalg.lstsq(F, y, rcond=None)
    return SurrogateModel(kind="poly2", X=X, y=y, coef=coef)


def fit_idw(X: np.ndarray, y: np.ndarray) -> SurrogateModel:
    return SurrogateModel(kind="idw", X=np.asarray(X, dtype=float), y=np.asarray(y, dtype=float).ravel())


def _idw_predict(X: np.ndarray, y: np.ndarray, x: np.ndarray, power: float = 2.0) -> float:
    d = np.linalg.norm(X - x, axis=1)
    if np.any(d < 1e-12):
        return float(y[np.argmin(d)])
    w = 1.0 / (d ** power)
    return float(np.sum(w * y) / np.sum(w))


def r2_score(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    y_true = np.asarray(y_true, dtype=float)
    y_pred = np.asarray(y_pred, dtype=float)
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    if ss_tot < 1e-15:
        return 1.0
    return float(1.0 - ss_res / ss_tot)
