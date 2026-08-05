"""Priority arbitration among chassis controllers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import numpy as np

from .controller_state import ActuatorCommands, ControllerState
from .driver_request import DriverInputs
from .sensor_model import SensorReading
from .abs_controller import ABSController
from .traction_control import TractionControl
from .esc_controller import ESCController
from .yaw_controller import YawController
from .brake_force_distribution import EBDController
from .launch_controller import LaunchController
from .hill_hold import HillHold
from .actuator_limits import ActuatorLimits, apply_limits


class ControllerPriority(IntEnum):
    """Higher wins on conflicts."""
    DRIVER = 0
    EBD = 1
    HILL_HOLD = 2
    LAUNCH = 3
    TC = 4
    ABS = 5
    ESC = 6
    YAW = 5


@dataclass
class ControllerManager:
    abs: ABSController
    tc: TractionControl
    esc: ESCController
    yaw: YawController
    ebd: EBDController
    launch: LaunchController
    hill: HillHold
    limits: ActuatorLimits

    def step(
        self,
        sensors: SensorReading,
        driver: DriverInputs,
        dt: float,
        *,
        abs_on: bool = True,
        tc_on: bool = True,
        esc_on: bool = True,
    ) -> tuple[ActuatorCommands, ControllerState]:
        st = ControllerState()
        cmd = ActuatorCommands(
            throttle=driver.throttle,
            brake_pressures=np.ones(4) * driver.brake,
            engine_torque_limit=1.0,
            tv_request=0.0,
            clutch=driver.clutch,
            gear_request=driver.gear_request,
        )

        # --- EBD base distribution ---
        self.ebd.enabled = True
        ebd_p, st.ebd_active = self.ebd.step(sensors, driver.brake)
        cmd.brake_pressures = ebd_p

        # --- Hill hold ---
        hp, st.hill_hold_active = self.hill.step(
            sensors, driver.brake, driver.throttle, driver.hill_hold_request
        )
        if st.hill_hold_active:
            cmd.brake_pressures = np.maximum(cmd.brake_pressures, hp)

        # --- Launch ---
        self.launch.enabled = True
        thr_l, clutch_l, st.launch_active = self.launch.step(
            sensors, driver.launch_request, driver.throttle, dt
        )
        if st.launch_active:
            cmd.throttle = thr_l
            cmd.clutch = min(cmd.clutch, clutch_l)

        # --- Traction control ---
        self.tc.enabled = tc_on
        tlim, brake_nudge, st.tc_active = self.tc.step(sensors, cmd.throttle, dt)
        if st.tc_active:
            cmd.engine_torque_limit = min(cmd.engine_torque_limit, tlim)
            # Light brake on spinning driven wheels (rear)
            if brake_nudge > 0:
                cmd.brake_pressures[2:] = np.maximum(
                    cmd.brake_pressures[2:], brake_nudge
                )

        # --- ABS (highest priority on brake pressures) ---
        self.abs.enabled = abs_on
        abs_p, abs_active, mu = self.abs.step(sensors, float(np.max(cmd.brake_pressures)), dt)
        st.abs_active = abs_active
        st.mu_est = mu
        if abs_on and driver.brake > 0.05:
            # ABS modulates each channel down from demand
            cmd.brake_pressures = np.minimum(cmd.brake_pressures, abs_p)
            # If ABS inactive on channel, keep EBD demand
            for i in range(4):
                if not abs_active[i]:
                    cmd.brake_pressures[i] = ebd_p[i] if not st.hill_hold_active else max(ebd_p[i], hp[i])

        # --- ESC ---
        self.esc.enabled = esc_on
        esc_brakes, esc_tlim, esc_tv, st.esc_active = self.esc.step(sensors, dt)
        if st.esc_active:
            cmd.brake_pressures = np.maximum(cmd.brake_pressures, esc_brakes)
            cmd.engine_torque_limit = min(cmd.engine_torque_limit, esc_tlim)
            cmd.tv_request += esc_tv

        # --- Yaw fine control ---
        r_ref = self.esc.reference_yaw(sensors.vx, sensors.steer)
        tv_y, st.yaw_error = self.yaw.step(sensors, r_ref, dt)
        cmd.tv_request += tv_y

        cmd = apply_limits(cmd, self.limits)
        return cmd, st
