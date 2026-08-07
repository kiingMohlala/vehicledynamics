"""Synthetic sensor generation (IMU, GPS, wheel speeds, suspension) with noise."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass
class SensorConfig:
    imu_noise_std: float = 0.02
    gps_noise_std: float = 0.5
    wheel_speed_noise_std: float = 0.05
    susp_noise_std: float = 0.001
    seed: int = 0


@dataclass
class SensorExporter:
    config: SensorConfig | None = None

    def __post_init__(self) -> None:
        self.cfg = self.config or SensorConfig()
        self.rng = np.random.default_rng(self.cfg.seed)

    def imu(self, ax: float, ay: float, az: float, gx: float = 0.0, gy: float = 0.0, gz: float = 0.0) -> dict[str, float]:
        n = self.cfg.imu_noise_std
        return {
            "ax": ax + float(self.rng.normal(0, n)),
            "ay": ay + float(self.rng.normal(0, n)),
            "az": az + float(self.rng.normal(0, n)),
            "gx": gx + float(self.rng.normal(0, n * 0.1)),
            "gy": gy + float(self.rng.normal(0, n * 0.1)),
            "gz": gz + float(self.rng.normal(0, n * 0.1)),
        }

    def gps(self, x: float, y: float, z: float = 0.0) -> dict[str, float]:
        n = self.cfg.gps_noise_std
        return {
            "x": x + float(self.rng.normal(0, n)),
            "y": y + float(self.rng.normal(0, n)),
            "z": z + float(self.rng.normal(0, n * 0.2)),
        }

    def wheel_speeds(self, speeds: list[float] | np.ndarray) -> list[float]:
        n = self.cfg.wheel_speed_noise_std
        return [float(w) + float(self.rng.normal(0, n)) for w in speeds]

    def suspension_travel(self, travels: list[float] | np.ndarray) -> list[float]:
        n = self.cfg.susp_noise_std
        return [float(z) + float(self.rng.normal(0, n)) for z in travels]

    def export(self, state: dict[str, Any]) -> dict[str, Any]:
        imu = self.imu(
            float(state.get("ax", 0.0)),
            float(state.get("ay", 0.0)),
            float(state.get("az", 9.81)),
            gz=float(state.get("yaw_rate", 0.0)),
        )
        gps = self.gps(float(state.get("x", 0.0)), float(state.get("y", 0.0)), float(state.get("z", 0.0)))
        ws = state.get("wheel_speed", [state.get("vx", 0.0)] * 4)
        return {
            "imu": imu,
            "gps": gps,
            "wheel_speeds": self.wheel_speeds(ws),
            "suspension_travel": self.suspension_travel(state.get("suspension_travel", [0, 0, 0, 0])),
        }
