"""
Transmission coordinator: engine → clutch → gearbox → final drive → wheel torque.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.powertrain.engine import EngineState

from .gear_ratios import GearRatios, default_ratios
from .clutch import Clutch
from .gearbox import Gearbox, GearboxType
from .shift_controller import ShiftController, ShiftPhase
from .shift_strategies import SequentialStrategy, ManualStrategy, ShiftStrategy
from .launch_control import LaunchControl
from .drivetrain_interface import DrivetrainOutput


@dataclass
class TransmissionConfig:
    enabled: bool = True
    gearbox: str = "sequential"   # sequential | manual
    final_drive: float = 3.90
    efficiency: float = 0.95
    initial_gear: int = 0
    # Phase 14.2H.2: explicit ratios; if provided, replace default_ratios() gears
    gear_ratios: list | None = None


@dataclass
class TransmissionState:
    gear: int = 0
    clutch_engagement: float = 0.0
    clutch_slip: float = 0.0
    clutch_torque: float = 0.0
    clutch_temp_C: float = 80.0
    gearbox_rpm: float = 0.0
    wheel_torque: float = 0.0
    shift_phase: str = "idle"
    shift_active: bool = False
    ignition_cut: bool = False
    locked: bool = False
    output: DrivetrainOutput = field(default_factory=DrivetrainOutput)


class TransmissionSolver:
    def __init__(self, config: TransmissionConfig | None = None):
        self.cfg = config or TransmissionConfig()
        ratios = default_ratios(self.cfg.final_drive)
        ratios.efficiency = self.cfg.efficiency
        # Authoritative path: explicit gear vector overrides library defaults
        if self.cfg.gear_ratios is not None and len(self.cfg.gear_ratios) > 1:
            ratios.gears = list(self.cfg.gear_ratios)
        gtype = (
            GearboxType.SEQUENTIAL
            if self.cfg.gearbox.lower().startswith("seq")
            else GearboxType.MANUAL
        )
        self.gearbox = Gearbox(ratios=ratios, gtype=gtype, current_gear=self.cfg.initial_gear)
        self.clutch = Clutch()
        self.shift = ShiftController()
        self.shift.reset(self.cfg.initial_gear)
        self.strategy: ShiftStrategy = (
            SequentialStrategy() if gtype == GearboxType.SEQUENTIAL else ManualStrategy()
        )
        self.launch = LaunchControl()
        self._omega_gb = 0.0  # gearbox input shaft speed
        self.state = TransmissionState(gear=self.cfg.initial_gear)

    def reset(self) -> None:
        self.clutch.reset()
        self.shift.reset(self.cfg.initial_gear)
        self.gearbox.current_gear = self.cfg.initial_gear
        self._omega_gb = 0.0
        self.state = TransmissionState(gear=self.cfg.initial_gear)

    def step(
        self,
        engine_state: EngineState,
        clutch: float = 1.0,
        requested_gear: int = 0,
        dt: float = 0.01,
        *,
        omega_wheel: float = 0.0,
        launch: bool = False,
    ) -> TransmissionState:
        """
        clutch: 1 = fully engaged, 0 = disengaged (driver input).
        requested_gear: absolute (manual) or relative ±1 (sequential).
        omega_wheel: optional wheel angular speed for gearbox input kinematics.
        """
        cfg = self.cfg
        if not cfg.enabled:
            self.state = TransmissionState()
            return self.state

        n_fwd = self.gearbox.ratios.n_forward
        target = self.strategy.resolve(
            requested_gear, self.shift.state.current_gear, n_fwd
        )
        if target != self.shift.state.current_gear and not self.shift.state.in_progress:
            self.shift.request(target)

        # Launch control may override clutch
        eng_rpm = engine_state.rpm
        eng_omega = engine_state.omega
        eng_tq = engine_state.torque_output

        lc = self.launch.step(eng_rpm, dt, launch)
        engagement = float(np.clip(clutch, 0.0, 1.0))
        if lc.active:
            engagement = min(engagement, lc.clutch_cmd)

        # Shift machine
        sh = self.shift.step(dt, eng_omega, self._omega_gb, engagement)
        if sh.auto_clutch is not None:
            engagement = sh.auto_clutch
        if sh.ignition_cut:
            eng_tq = 0.0

        # Gearbox gear from shift state
        self.gearbox.set_gear(sh.current_gear)

        # Gearbox input speed from wheels when in gear
        if sh.current_gear != 0 and abs(omega_wheel) > 1e-6:
            self._omega_gb = self.gearbox.input_omega_from_output(omega_wheel)
        elif sh.current_gear == 0:
            # Free spinning input follows engine through clutch partially
            self._omega_gb = 0.85 * self._omega_gb + 0.15 * eng_omega * engagement

        cl = self.clutch.step(
            engagement, eng_omega, self._omega_gb, eng_tq, dt
        )

        # Update gearbox input speed when slipping (driven by clutch)
        if not cl.locked and abs(cl.torque) > 1e-6:
            # Simple 1st-order approach of gb shaft toward engine
            self._omega_gb += (cl.torque / 0.15) * dt  # small input inertia
            self._omega_gb = max(self._omega_gb, 0.0)

        wheel_tq, _ = self.gearbox.apply(cl.torque, self._omega_gb)
        gb_rpm = self._omega_gb * 60.0 / (2.0 * np.pi)

        out = DrivetrainOutput(
            wheel_torque=wheel_tq,
            gearbox_rpm=gb_rpm,
            clutch_slip=cl.omega_slip,
            current_gear=sh.current_gear,
            shift_active=sh.in_progress,
        )
        self.state = TransmissionState(
            gear=sh.current_gear,
            clutch_engagement=cl.engagement,
            clutch_slip=cl.omega_slip,
            clutch_torque=cl.torque,
            clutch_temp_C=cl.temp_C,
            gearbox_rpm=gb_rpm,
            wheel_torque=wheel_tq,
            shift_phase=sh.phase.value,
            shift_active=sh.in_progress,
            ignition_cut=sh.ignition_cut,
            locked=cl.locked,
            output=out,
        )
        return self.state
