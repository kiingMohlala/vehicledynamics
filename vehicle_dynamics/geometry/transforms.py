"""Rigid transforms, rotation matrices, quaternions."""
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .vector import as_vec3, normalize


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def euler_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    return rot_x(rx) @ rot_y(ry) @ rot_z(rz)


def quat_from_axis_angle(axis: ArrayLike, angle: float) -> np.ndarray:
    ax = normalize(axis)
    s = np.sin(angle / 2)
    return np.array([np.cos(angle / 2), *(ax * s)], dtype=float)


def quat_to_matrix(q: ArrayLike) -> np.ndarray:
    q = np.asarray(q, dtype=float).ravel()
    w, x, y, z = q / (np.linalg.norm(q) + 1e-15)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def transform_points(points: ArrayLike, R: np.ndarray, t: ArrayLike) -> np.ndarray:
    P = np.asarray(points, dtype=float)
    t = as_vec3(t)
    if P.ndim == 1:
        return R @ P + t
    return (R @ P.T).T + t


def rigid_inverse(R: np.ndarray, t: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
    Rt = R.T
    return Rt, -Rt @ as_vec3(t)
