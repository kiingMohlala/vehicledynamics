"""Phase 9 – Aerodynamics (foundation, closed-loop, devices)."""

from .coefficients import AeroConfig, AeroCoefficients
from .aero_model import AeroState, compute_aero_loads
from .ride_height import RideHeightState, ride_height_factors
from .aero_map import AeroMap, build_default_map
from .aero_solver import AeroResult, solve_aero
from .aero_report import format_aero_report
from .vehicle_interface import VehicleAeroInput, ride_from_pitch_heave, ride_from_vehicle_input
from .coupling import CoupledAxleLoads, couple_aero_to_tires, static_axle_loads
from .closed_loop import ClosedLoopAero, ClosedLoopAeroResult, PitchHeaveState, PitchHeaveParams
from .aero_devices import AeroDeviceConfig
from .aero_device_solver import AeroDeviceSolver, DeviceAeroResult, DeviceBreakdown
from .aero_device_report import format_device_report
from .drs import DRSController, DRSState, DRSParams
from .active_aero import ActiveAeroController, ActiveAeroMode, ActiveAeroParams

__all__ = [
    "AeroConfig", "AeroCoefficients", "AeroState", "compute_aero_loads",
    "RideHeightState", "ride_height_factors", "AeroMap", "build_default_map",
    "AeroResult", "solve_aero", "format_aero_report",
    "VehicleAeroInput", "ride_from_pitch_heave", "ride_from_vehicle_input",
    "CoupledAxleLoads", "couple_aero_to_tires", "static_axle_loads",
    "ClosedLoopAero", "ClosedLoopAeroResult", "PitchHeaveState", "PitchHeaveParams",
    "AeroDeviceConfig", "AeroDeviceSolver", "DeviceAeroResult", "DeviceBreakdown",
    "format_device_report", "DRSController", "DRSState", "DRSParams",
    "ActiveAeroController", "ActiveAeroMode", "ActiveAeroParams",
]

# CFD maps: vehicle_dynamics.aerodynamics.cfd
