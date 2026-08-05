"""Virtual vehicle sensors (from plant state dict / attributes)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SensorReading:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    steer: float = 0.0
    engine_rpm: float = 0.0
    wheel_omega: np.ndarray = None  # type: ignore
    slip_ratio: np.ndarray = None  # type: ignore
    slip_angle: np.ndarray = None  # type: ignore
    brake_pressure: np.ndarray = None  # type: ignore

    def __post_init__(self) -> None:
        if self.wheel_omega is None:
            self.wheel_omega = np.zeros(4)
        if self.slip_ratio is None:
            self.slip_ratio = np.zeros(4)
        if self.slip_angle is None:
            self.slip_angle = np.zeros(4)
        if self.brake_pressure is None:
            self.brake_pressure = np.zeros(4)


class SensorModel:
    """Extract sensor signals from a loose vehicle_state mapping."""

    def read(self, vehicle_state: dict, driver_steer: float = 0.0) -> SensorReading:
        def g(key, default=0.0):
            return float(vehicle_state.get(key, default))

        wo = vehicle_state.get("wheel_omega", [0, 0, 0, 0])
        sr = vehicle_state.get("slip_ratio", [0, 0, 0, 0])
        sa = vehicle_state.get("slip_angle", [0, 0, 0, 0])
        bp = vehicle_state.get("brake_pressure", [0, 0, 0, 0])
        return SensorReading(
            vx=g("vx"),
            vy=g("vy"),
            yaw_rate=g("yaw_rate", g("r")),
            ax=g("ax"),
            ay=g("ay"),
            steer=g("steer", driver_steer),
            engine_rpm=g("engine_rpm"),
            wheel_omega=np.asarray(wo, dtype=float).reshape(4,),
            slip_ratio=np.asarray(sr, dtype=float).reshape(4,),
            slip_angle=np.asarray(sa, dtype=float).reshape(4,),
            brake_pressure=np.asarray(bp, dtype=float).reshape(4,),
        )
