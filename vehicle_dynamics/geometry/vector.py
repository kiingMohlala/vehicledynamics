"""3D vector math."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def as_vec3(v: ArrayLike) -> np.ndarray:
    a = np.asarray(v, dtype=float).ravel()
    if a.size != 3:
        raise ValueError("Expected 3-vector")
    return a


def norm(v: ArrayLike) -> float:
    return float(np.linalg.norm(as_vec3(v)))


def normalize(v: ArrayLike) -> np.ndarray:
    a = as_vec3(v)
    n = np.linalg.norm(a)
    if n < 1e-15:
        return np.zeros(3)
    return a / n


def dot(a: ArrayLike, b: ArrayLike) -> float:
    return float(np.dot(as_vec3(a), as_vec3(b)))


def cross(a: ArrayLike, b: ArrayLike) -> np.ndarray:
    return np.cross(as_vec3(a), as_vec3(b))


def distance(a: ArrayLike, b: ArrayLike) -> float:
    return norm(as_vec3(a) - as_vec3(b))


def lerp(a: ArrayLike, b: ArrayLike, t: float) -> np.ndarray:
    a, b = as_vec3(a), as_vec3(b)
    return (1.0 - t) * a + t * b
