"""Subsystem configuration blocks."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TireConfig:
    model: str = "dugoff"          # dugoff | pacejka
    mu: float = 1.0
    Cx: float = 80000.0
    Cy: float = 80000.0
    radius_m: float = 0.32
    load_sensitive: bool = False
    relaxation: bool = False


@dataclass
class SuspensionConfig:
    ks_front: float = 28000.0
    ks_rear: float = 32000.0
    cs_front: float = 2500.0
    cs_rear: float = 2800.0
    motion_ratio_f: float = 1.0
    motion_ratio_r: float = 1.0
    camber_static_deg: float = -1.0
    toe_static_deg: float = 0.0
    use_geometry: bool = False


@dataclass
class BrakeConfig:
    max_torque_Nm: float = 3000.0
    bias_front: float = 0.60
    abs_enabled: bool = True
    rotor_mass_kg: float = 8.0


@dataclass
class AeroConfigBlock:
    enabled: bool = True
    Cd: float = 0.34
    Cl_front: float = -0.45
    Cl_rear: float = -0.70
    frontal_area_m2: float = 1.90
    rho: float = 1.225


@dataclass
class PowertrainConfigBlock:
    architecture: str = "ice"      # ice | parallel | series | ev
    peak_power_kw: float = 300.0
    redline_rpm: float = 7500.0
    idle_rpm: float = 900.0
    final_drive: float = 3.90
    gearbox: str = "sequential"
    hybrid_battery_kwh: float = 0.0
    motor_peak_kw: float = 0.0
    differential: str = "open"
    # Phase 14.2H.2 — explicit gear vector; index 0 unused, 1..n forward
    # Empty list → transmission may use library default (historical path only)
    gear_ratios: list = None  # type: ignore
    transmission_efficiency: float = 0.95

    def __post_init__(self):
        if self.gear_ratios is None:
            self.gear_ratios = []


@dataclass
class ControlsConfigBlock:
    abs: bool = True
    tc: bool = True
    esc: bool = True
    strategy_enabled: bool = True
    drive_mode: str = "normal"


@dataclass
class DriverConfigBlock:
    mode: str = "pure_pursuit"
    look_ahead_m: float = 12.0


@dataclass
class ChassisConfig:
    compliance_enabled: bool = False
    fem_enabled: bool = False
    nonlinear_frame: bool = False


@dataclass
class SubsystemBundle:
    tire: TireConfig = field(default_factory=TireConfig)
    suspension: SuspensionConfig = field(default_factory=SuspensionConfig)
    brakes: BrakeConfig = field(default_factory=BrakeConfig)
    aero: AeroConfigBlock = field(default_factory=AeroConfigBlock)
    powertrain: PowertrainConfigBlock = field(default_factory=PowertrainConfigBlock)
    controls: ControlsConfigBlock = field(default_factory=ControlsConfigBlock)
    driver: DriverConfigBlock = field(default_factory=DriverConfigBlock)
    chassis: ChassisConfig = field(default_factory=ChassisConfig)
    extras: dict[str, Any] = field(default_factory=dict)
