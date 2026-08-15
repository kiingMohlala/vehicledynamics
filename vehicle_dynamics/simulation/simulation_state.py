"""Unified simulation state container."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    wheel_omega: np.ndarray = field(default_factory=lambda: np.zeros(4))
    slip_ratio: np.ndarray = field(default_factory=lambda: np.zeros(4))
    slip_angle: np.ndarray = field(default_factory=lambda: np.zeros(4))
    Fz: np.ndarray = field(default_factory=lambda: np.array([3500.0, 3500.0, 3500.0, 3500.0]))
    engine_rpm: float = 900.0
    gear: int = 1
    fuel_g: float = 0.0
    ride_h_front: float = 0.08
    ride_h_rear: float = 0.10
    downforce: float = 0.0
    drag: float = 0.0

    def as_sensor_dict(self) -> dict:
        return {
            "vx": self.vx,
            "vy": self.vy,
            "yaw_rate": self.yaw_rate,
            "ax": self.ax,
            "ay": self.ay,
            "engine_rpm": self.engine_rpm,
            "wheel_omega": self.wheel_omega.tolist(),
            "slip_ratio": self.slip_ratio.tolist(),
            "slip_angle": self.slip_angle.tolist(),
            "steer": 0.0,
            "x": self.x,
            "y": self.y,
            "psi": self.psi,
        }

    def as_pose_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "psi": self.psi,
            "vx": self.vx,
            "ax": self.ax,
            "ay": self.ay,
            "yaw_rate": self.yaw_rate,
        }


@dataclass
class SimulationState:
    time: float = 0.0
    vehicle: VehicleState = field(default_factory=VehicleState)
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    engine_torque: float = 0.0
    wheel_torque_L: float = 0.0
    wheel_torque_R: float = 0.0
    gear: int = 1
    mu_scale: float = 1.0
    crosswind: float = 0.0  # legacy lateral wind proxy (m/s); maps to wind_vy if wind unset
    wind_vx: float = 0.0    # Phase 14.3 body-frame wind +x (m/s)
    wind_vy: float = 0.0    # Phase 14.3 body-frame wind +y (m/s)
    rain: bool = False
