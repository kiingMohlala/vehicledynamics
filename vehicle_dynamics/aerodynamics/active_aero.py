"""Active aero mode controller (rear wing + DRS)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np


class ActiveAeroMode(str, Enum):
    DISABLED = "disabled"
    CORNERING = "cornering"
    BRAKING = "braking"
    TOP_SPEED = "top_speed"
    AUTOMATIC = "automatic"


@dataclass
class ActiveAeroParams:
    cornering_alpha: float = 0.18      # rad rear wing
    braking_alpha: float = 0.22
    top_speed_alpha: float = 0.05
    default_alpha: float = 0.12
    ay_corner_threshold: float = 3.0   # m/s²
    brake_threshold: float = 0.15      # normalized brake 0-1
    top_speed_threshold: float = 55.0  # m/s
    balance_target: float = 0.40       # front DF fraction target (info)


@dataclass
class ActiveAeroCommand:
    mode: ActiveAeroMode
    rear_wing_alpha: float
    drs_open: bool
    balance_target: float


class ActiveAeroController:
    def __init__(self, params: ActiveAeroParams | None = None):
        self.params = params or ActiveAeroParams()
        self.mode = ActiveAeroMode.AUTOMATIC

    def set_mode(self, mode: ActiveAeroMode) -> None:
        self.mode = mode

    def update(
        self,
        speed: float,
        ay: float = 0.0,
        brake: float = 0.0,
    ) -> ActiveAeroCommand:
        p = self.params
        mode = self.mode

        if mode == ActiveAeroMode.DISABLED:
            return ActiveAeroCommand(mode, p.default_alpha, False, p.balance_target)

        if mode == ActiveAeroMode.AUTOMATIC:
            if brake >= p.brake_threshold:
                mode = ActiveAeroMode.BRAKING
            elif abs(ay) >= p.ay_corner_threshold:
                mode = ActiveAeroMode.CORNERING
            elif speed >= p.top_speed_threshold:
                mode = ActiveAeroMode.TOP_SPEED
            else:
                mode = ActiveAeroMode.CORNERING  # default to grip

        if mode == ActiveAeroMode.BRAKING:
            return ActiveAeroCommand(mode, p.braking_alpha, False, p.balance_target)
        if mode == ActiveAeroMode.CORNERING:
            return ActiveAeroCommand(mode, p.cornering_alpha, False, p.balance_target)
        if mode == ActiveAeroMode.TOP_SPEED:
            return ActiveAeroCommand(mode, p.top_speed_alpha, True, p.balance_target)
        return ActiveAeroCommand(mode, p.default_alpha, False, p.balance_target)
