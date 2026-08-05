"""Actuator saturation / rate limits."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .controller_state import ActuatorCommands


@dataclass
class ActuatorLimits:
    throttle_max: float = 1.0
    brake_max: float = 1.0
    tv_max: float = 500.0
    torque_limit_min: float = 0.0
    torque_limit_max: float = 1.0


def apply_limits(cmd: ActuatorCommands, lim: ActuatorLimits) -> ActuatorCommands:
    cmd.throttle = float(np.clip(cmd.throttle, 0.0, lim.throttle_max))
    cmd.brake_pressures = np.clip(cmd.brake_pressures, 0.0, lim.brake_max)
    cmd.engine_torque_limit = float(
        np.clip(cmd.engine_torque_limit, lim.torque_limit_min, lim.torque_limit_max)
    )
    cmd.tv_request = float(np.clip(cmd.tv_request, -lim.tv_max, lim.tv_max))
    cmd.clutch = float(np.clip(cmd.clutch, 0.0, 1.0))
    return cmd
