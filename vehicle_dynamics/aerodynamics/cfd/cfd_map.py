"""Aerodynamic sample and N-D map structures."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class AeroSample:
    """One operating-point aero measurement (CFD / WT / track)."""

    speed: float = 40.0          # m/s
    h_front: float = 0.08        # m
    h_rear: float = 0.10
    pitch: float = 0.0           # rad
    yaw: float = 0.0             # rad
    roll: float = 0.0
    wing_angle: float = 0.12     # rad rear wing
    drs: float = 0.0             # 0 closed … 1 open

    Cd: float = 0.34
    Cl_front: float = -0.45
    Cl_rear: float = -0.70
    Cy: float = 0.0
    Cm_pitch: float = 0.0
    Cn_yaw: float = 0.0
    x_cop: float = 0.0           # m from mid-wheelbase, + aft

    source: str = "synthetic"
    meta: dict = field(default_factory=dict)

    def state_vector(self) -> np.ndarray:
        return np.array([
            self.speed, self.h_front, self.h_rear, self.pitch,
            self.yaw, self.roll, self.wing_angle, self.drs,
        ], dtype=float)

    def coeff_vector(self) -> np.ndarray:
        return np.array([
            self.Cd, self.Cl_front, self.Cl_rear, self.Cy,
            self.Cm_pitch, self.Cn_yaw, self.x_cop,
        ], dtype=float)


@dataclass
class AeroMapND:
    """Collection of samples forming a lookup database."""

    samples: list[AeroSample] = field(default_factory=list)
    name: str = "default"
    axes_labels: tuple[str, ...] = (
        "speed", "h_front", "h_rear", "pitch", "yaw", "roll", "wing_angle", "drs"
    )

    def __len__(self) -> int:
        return len(self.samples)

    def state_matrix(self) -> np.ndarray:
        if not self.samples:
            return np.zeros((0, 8))
        return np.vstack([s.state_vector() for s in self.samples])

    def coeff_matrix(self) -> np.ndarray:
        if not self.samples:
            return np.zeros((0, 7))
        return np.vstack([s.coeff_vector() for s in self.samples])

    def bounds(self) -> dict[str, tuple[float, float]]:
        X = self.state_matrix()
        if X.size == 0:
            return {}
        out = {}
        for i, lab in enumerate(self.axes_labels):
            out[lab] = (float(X[:, i].min()), float(X[:, i].max()))
        return out
