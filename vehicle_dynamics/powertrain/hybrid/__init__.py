"""Phase 10.4 – Hybrid & electric powertrain."""

from .battery import Battery, BatteryConfig, BatteryState
from .battery_thermal import BatteryThermal
from .battery_degradation import BatteryDegradation
from .inverter import Inverter
from .motor_map import MotorMap, default_motor_map
from .motor import ElectricMotor, MotorConfig, MotorState
from .regen_braking import RegenBraking, RegenConfig
from .torque_blending import TorqueBlender, BlendMode
from .energy_manager import EnergyManager, EnergyMode
from .hybrid_controller import HybridController
from .awd_distribution import AWDDistributor
from .charging import Charger
from .hybrid_state import HybridState
from .hybrid_solver import HybridConfig, HybridSolver
from .hybrid_report import format_hybrid_report

__all__ = [
    "Battery", "BatteryConfig", "BatteryState",
    "BatteryThermal", "BatteryDegradation",
    "Inverter", "MotorMap", "default_motor_map",
    "ElectricMotor", "MotorConfig", "MotorState",
    "RegenBraking", "RegenConfig",
    "TorqueBlender", "BlendMode",
    "EnergyManager", "EnergyMode",
    "HybridController", "AWDDistributor", "Charger",
    "HybridState", "HybridConfig", "HybridSolver",
    "format_hybrid_report",
]
