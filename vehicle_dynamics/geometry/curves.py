"""Basic curves: line, arc, Bézier."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike

from .vector import as_vec3, distance, lerp, normalize


@dataclass
class Line:
    p0: np.ndarray
    p1: np.ndarray

    def __post_init__(self) -> None:
        self.p0 = as_vec3(self.p0)
        self.p1 = as_vec3(self.p1)

    def evaluate(self, t: float) -> np.ndarray:
        return lerp(self.p0, self.p1, t)

    def tangent(self, t: float = 0.0) -> np.ndarray:
        return normalize(self.p1 - self.p0)

    def length(self) -> float:
        return distance(self.p0, self.p1)

    def sample(self, n: int = 20) -> np.ndarray:
        ts = np.linspace(0, 1, n)
        return np.array([self.evaluate(t) for t in ts])


@dataclass
class Arc:
    center: np.ndarray
    radius: float
    start_angle: float
    end_angle: float
    normal: np.ndarray = None  # type: ignore

    def __post_init__(self) -> None:
        self.center = as_vec3(self.center)
        if self.normal is None:
            self.normal = np.array([0.0, 0.0, 1.0])
        else:
            self.normal = normalize(self.normal)

    def evaluate(self, t: float) -> np.ndarray:
        a = self.start_angle + t * (self.end_angle - self.start_angle)
        # circle in XY for simplicity, rotated by normal≈Z default
        return self.center + self.radius * np.array([np.cos(a), np.sin(a), 0.0])

    def tangent(self, t: float) -> np.ndarray:
        a = self.start_angle + t * (self.end_angle - self.start_angle)
        return normalize(np.array([-np.sin(a), np.cos(a), 0.0]))

    def length(self) -> float:
        return abs(self.end_angle - self.start_angle) * self.radius

    def sample(self, n: int = 40) -> np.ndarray:
        return np.array([self.evaluate(t) for t in np.linspace(0, 1, n)])


@dataclass
class BezierCurve:
    control_points: np.ndarray  # (n, 3)

    def __post_init__(self) -> None:
        self.control_points = np.asarray(self.control_points, dtype=float)
        if self.control_points.ndim != 2 or self.control_points.shape[1] != 3:
            raise ValueError("control_points must be (n,3)")

    @property
    def degree(self) -> int:
        return self.control_points.shape[0] - 1

    def evaluate(self, t: float) -> np.ndarray:
        pts = self.control_points.copy()
        n = len(pts)
        for r in range(1, n):
            for i in range(n - r):
                pts[i] = (1 - t) * pts[i] + t * pts[i + 1]
        return pts[0]

    def tangent(self, t: float, eps: float = 1e-6) -> np.ndarray:
        t0, t1 = max(0.0, t - eps), min(1.0, t + eps)
        return normalize(self.evaluate(t1) - self.evaluate(t0))

    def length(self, n: int = 100) -> float:
        pts = self.sample(n)
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))

    def sample(self, n: int = 50) -> np.ndarray:
        return np.array([self.evaluate(t) for t in np.linspace(0, 1, n)])

    def subdivide(self, t: float = 0.5) -> tuple["BezierCurve", "BezierCurve"]:
        """de Casteljau split."""
        pts = self.control_points.copy()
        left, right = [pts[0]], [pts[-1]]
        n = len(pts)
        for r in range(1, n):
            for i in range(n - r):
                pts[i] = (1 - t) * pts[i] + t * pts[i + 1]
            left.append(pts[0].copy())
            right.append(pts[n - r - 1].copy())
        return BezierCurve(np.array(left)), BezierCurve(np.array(right[::-1]))
