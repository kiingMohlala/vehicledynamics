"""Engine configuration and state containers."""

from __future__ import annotations

from dataclasses import dataclass, field
from .engine_map import EngineMap, default_na_map
from .rev_limiter import LimitMode


@dataclass
class EngineConfig:
    enabled: bool = True
    idle_rpm: float = 900.0
    redline_rpm: float = 7500.0
    stall_rpm: float = 400.0
    inertia: float = 0.25
    peak_torque: float = 400.0
    peak_torque_rpm: float = 4500.0
    limiter_mode: LimitMode = LimitMode.SOFT
    soft_start_rpm: float = 7300.0
    friction_coeff: float = 0.01  # N·m·s/rad viscous
    map: EngineMap | None = None

    def get_map(self) -> EngineMap:
        if self.map is None:
            self.map = default_na_map(
                idle_rpm=self.idle_rpm,
                redline_rpm=self.redline_rpm,
                peak_torque=self.peak_torque,
                peak_torque_rpm=self.peak_torque_rpm,
            )
        return self.map


@dataclass
class EngineState:
    rpm: float = 900.0
    omega: float = 0.0          # rad/s
    torque_indicated: float = 0.0
    torque_brake: float = 0.0
    torque_output: float = 0.0
    power_kw: float = 0.0
    throttle: float = 0.0
    limiter_factor: float = 1.0
