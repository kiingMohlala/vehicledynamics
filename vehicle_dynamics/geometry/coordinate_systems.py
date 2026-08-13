"""Local / global coordinate frames."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .vector import as_vec3, normalize, cross
from .transforms import transform_points


@dataclass
class Frame:
    origin: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    z_axis: np.ndarray

    @classmethod
    def from_origin_z(cls, origin, z_dir, x_hint=(1.0, 0.0, 0.0)) -> "Frame":
        z = normalize(z_dir)
        x = normalize(np.cross(as_vec3(x_hint), z))
        if np.linalg.norm(x) < 1e-12:
            x = normalize(np.cross((0.0, 1.0, 0.0), z))
        y = cross(z, x)
        return cls(as_vec3(origin), x, y, z)

    @property
    def R(self) -> np.ndarray:
        return np.column_stack([self.x_axis, self.y_axis, self.z_axis])

    def to_global(self, local_pts: np.ndarray) -> np.ndarray:
        return transform_points(local_pts, self.R, self.origin)

    def to_local(self, global_pts: np.ndarray) -> np.ndarray:
        P = np.asarray(global_pts, dtype=float)
        if P.ndim == 1:
            return self.R.T @ (P - self.origin)
        return (self.R.T @ (P - self.origin).T).T
