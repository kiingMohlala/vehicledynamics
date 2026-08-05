"""Phase 10.1 – Clutch & gearbox transmission."""

from .gear_ratios import GearRatios, default_ratios
from .clutch_friction import ClutchFrictionParams, clutch_capacity
from .clutch import Clutch, ClutchState
from .synchronizer import Synchronizer, SyncState
from .gearbox import Gearbox, GearboxType
from .shift_controller import ShiftController, ShiftPhase, ShiftState
from .shift_strategies import ShiftStrategy, SequentialStrategy, ManualStrategy
from .launch_control import LaunchControl, LaunchState
from .drivetrain_interface import DrivetrainOutput
from .transmission_solver import TransmissionConfig, TransmissionSolver, TransmissionState
from .transmission_report import format_transmission_report

__all__ = [
    "GearRatios",
    "default_ratios",
    "ClutchFrictionParams",
    "clutch_capacity",
    "Clutch",
    "ClutchState",
    "Synchronizer",
    "SyncState",
    "Gearbox",
    "GearboxType",
    "ShiftController",
    "ShiftPhase",
    "ShiftState",
    "ShiftStrategy",
    "SequentialStrategy",
    "ManualStrategy",
    "LaunchControl",
    "LaunchState",
    "DrivetrainOutput",
    "TransmissionConfig",
    "TransmissionSolver",
    "TransmissionState",
    "format_transmission_report",
]
