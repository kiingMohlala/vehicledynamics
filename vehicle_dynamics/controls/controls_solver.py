"""
Controls layer entry point.
Controllers modify commands; physics is unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass

from .driver_request import DriverInputs
from .sensor_model import SensorModel
from .actuator_limits import ActuatorLimits
from .abs_controller import ABSController
from .traction_control import TractionControl
from .esc_controller import ESCController
from .yaw_controller import YawController
from .brake_force_distribution import EBDController
from .launch_controller import LaunchController
from .hill_hold import HillHold
from .controller_manager import ControllerManager
from .controller_state import ActuatorCommands, ControllerState


@dataclass
class ControlsConfig:
    enabled: bool = True
    abs_enabled: bool = True
    tc_enabled: bool = True
    esc_enabled: bool = True
    yaw_enabled: bool = True
    ebd_enabled: bool = True
    launch_enabled: bool = True
    hill_hold_enabled: bool = True


class ControlsSolver:
    def __init__(self, config: ControlsConfig | None = None):
        self.cfg = config or ControlsConfig()
        self.sensors = SensorModel()
        self.manager = ControllerManager(
            abs=ABSController(enabled=self.cfg.abs_enabled),
            tc=TractionControl(enabled=self.cfg.tc_enabled),
            esc=ESCController(enabled=self.cfg.esc_enabled),
            yaw=YawController(enabled=self.cfg.yaw_enabled),
            ebd=EBDController(enabled=self.cfg.ebd_enabled),
            launch=LaunchController(enabled=self.cfg.launch_enabled),
            hill=HillHold(enabled=self.cfg.hill_hold_enabled),
            limits=ActuatorLimits(),
        )
        self.last_commands = ActuatorCommands()
        self.last_state = ControllerState()

    def step(
        self,
        vehicle_state: dict,
        driver_inputs: DriverInputs,
        dt: float = 0.01,
    ) -> ActuatorCommands:
        """
        Returns actuator commands for powertrain / brakes / differential.
        If controls disabled, pass-through driver inputs.
        """
        if not self.cfg.enabled:
            cmd = ActuatorCommands(
                throttle=driver_inputs.throttle,
                brake_pressures=__import__("numpy").ones(4) * driver_inputs.brake,
                engine_torque_limit=1.0,
                tv_request=0.0,
                clutch=driver_inputs.clutch,
                gear_request=driver_inputs.gear_request,
            )
            self.last_commands = cmd
            self.last_state = ControllerState()
            return cmd

        reading = self.sensors.read(vehicle_state, driver_inputs.steer)
        cmd, st = self.manager.step(
            reading,
            driver_inputs,
            dt,
            abs_on=self.cfg.abs_enabled,
            tc_on=self.cfg.tc_enabled,
            esc_on=self.cfg.esc_enabled,
        )
        self.last_commands = cmd
        self.last_state = st
        return cmd
