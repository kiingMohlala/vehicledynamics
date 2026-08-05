"""Shared controller / actuator command state."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ActuatorCommands:
    throttle: float = 0.0
    brake_pressures: np.ndarray = field(default_factory=lambda: np.zeros(4))  # FL FR RL RR 0..1
    engine_torque_limit: float = 1.0   # scale 0..1 on engine torque
    tv_request: float = 0.0            # ΔT for differential TV (N·m)
    clutch: float = 1.0
    gear_request: int = 0
    ignition_cut: bool = False


@dataclass
class ControllerState:
    abs_active: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=bool))
    tc_active: bool = False
    esc_active: bool = False
    ebd_active: bool = False
    launch_active: bool = False
    hill_hold_active: bool = False
    yaw_error: float = 0.0
    mu_est: float = 1.0
