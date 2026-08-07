"""Phase 12.0 – Vehicle Architecture & Digital Twin Framework."""

from .geometry import GeometryConfig
from .mass_properties import MassProperties
from .configuration import (
    TireConfig, SuspensionConfig, BrakeConfig, AeroConfigBlock,
    PowertrainConfigBlock, ControlsConfigBlock, DriverConfigBlock,
    ChassisConfig, SubsystemBundle,
)
from .vehicle_definition import VehicleDefinition
from .vehicle_builder import VehicleBuilder, BuiltVehicle
from .vehicle_registry import VehicleRegistry, DEFAULT_VEHICLE_REGISTRY
from .subsystem_registry import SubsystemRegistry, DEFAULT_REGISTRY
from .presets import load_preset, list_presets
from .serialization import save_json, load_json, save_yaml, load_yaml
from .digital_twin import DigitalTwin, create_digital_twin
from .comparison import compare_definitions, compare_twins, ComparisonResult
from .report import format_vehicle_report

__all__ = [
    "GeometryConfig",
    "MassProperties",
    "TireConfig",
    "SuspensionConfig",
    "BrakeConfig",
    "AeroConfigBlock",
    "PowertrainConfigBlock",
    "ControlsConfigBlock",
    "DriverConfigBlock",
    "ChassisConfig",
    "SubsystemBundle",
    "VehicleDefinition",
    "VehicleBuilder",
    "BuiltVehicle",
    "VehicleRegistry",
    "DEFAULT_VEHICLE_REGISTRY",
    "SubsystemRegistry",
    "DEFAULT_REGISTRY",
    "load_preset",
    "list_presets",
    "save_json",
    "load_json",
    "save_yaml",
    "load_yaml",
    "DigitalTwin",
    "create_digital_twin",
    "compare_definitions",
    "compare_twins",
    "ComparisonResult",
    "format_vehicle_report",
]
