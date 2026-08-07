"""Factory presets for common vehicle classes."""
from __future__ import annotations

from .geometry import GeometryConfig
from .mass_properties import MassProperties
from .configuration import (
    SubsystemBundle, TireConfig, SuspensionConfig, BrakeConfig,
    AeroConfigBlock, PowertrainConfigBlock, ControlsConfigBlock,
)
from .vehicle_definition import VehicleDefinition


def generic_sedan() -> VehicleDefinition:
    return VehicleDefinition(
        name="generic_sedan",
        vehicle_class="passenger",
        description="Baseline passenger car",
        geometry=GeometryConfig(wheelbase_m=2.70, a_m=1.20, b_m=1.50, track_front_m=1.55),
        mass=MassProperties(mass_kg=1400.0, Iz_kgm2=2500.0),
    )


def formula_sae() -> VehicleDefinition:
    return VehicleDefinition(
        name="formula_sae",
        vehicle_class="formula",
        description="Lightweight Formula Student / FSAE prototype",
        geometry=GeometryConfig(
            wheelbase_m=1.60, a_m=0.80, b_m=0.80,
            track_front_m=1.20, track_rear_m=1.15,
            h_cg_m=0.30, wheel_radius_m=0.25,
            overall_length_m=2.8, overall_width_m=1.4, overall_height_m=1.0,
        ),
        mass=MassProperties(mass_kg=250.0, Iz_kgm2=120.0, fuel_mass_kg=5.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="pacejka", mu=1.4, radius_m=0.25),
            suspension=SuspensionConfig(ks_front=35000, ks_rear=40000, camber_static_deg=-2.0),
            brakes=BrakeConfig(max_torque_Nm=800, bias_front=0.55),
            aero=AeroConfigBlock(Cd=0.55, Cl_front=-0.8, Cl_rear=-1.2, frontal_area_m2=1.0),
            powertrain=PowertrainConfigBlock(peak_power_kw=60, redline_rpm=12000, final_drive=4.2),
            controls=ControlsConfigBlock(drive_mode="sport"),
        ),
    )


def gt3() -> VehicleDefinition:
    return VehicleDefinition(
        name="gt3_prototype",
        vehicle_class="gt",
        description="GT3-class race car",
        geometry=GeometryConfig(
            wheelbase_m=2.60, a_m=1.25, b_m=1.35,
            track_front_m=1.65, track_rear_m=1.62, h_cg_m=0.42,
            wheel_radius_m=0.33, overall_length_m=4.6, overall_width_m=2.0,
        ),
        mass=MassProperties(mass_kg=1280.0, Iz_kgm2=2100.0, fuel_mass_kg=80.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="pacejka", mu=1.6, Cx=120000, Cy=110000),
            suspension=SuspensionConfig(ks_front=90000, ks_rear=100000, camber_static_deg=-3.0),
            brakes=BrakeConfig(max_torque_Nm=4500, bias_front=0.58),
            aero=AeroConfigBlock(Cd=0.55, Cl_front=-1.2, Cl_rear=-2.0, frontal_area_m2=2.0),
            powertrain=PowertrainConfigBlock(peak_power_kw=400, redline_rpm=8500, differential="clutch_lsd"),
            controls=ControlsConfigBlock(drive_mode="race"),
        ),
    )


def hypercar() -> VehicleDefinition:
    return VehicleDefinition(
        name="hypercar",
        vehicle_class="hypercar",
        description="Hybrid hypercar digital twin",
        geometry=GeometryConfig(
            wheelbase_m=2.70, a_m=1.35, b_m=1.35,
            track_front_m=1.70, track_rear_m=1.68, h_cg_m=0.40,
            wheel_radius_m=0.34, overall_length_m=4.7, overall_width_m=2.1,
        ),
        mass=MassProperties(mass_kg=1450.0, Iz_kgm2=2400.0, fuel_mass_kg=40.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="pacejka", mu=1.7, load_sensitive=True, relaxation=True),
            suspension=SuspensionConfig(ks_front=120000, ks_rear=140000, use_geometry=True),
            brakes=BrakeConfig(max_torque_Nm=5000, bias_front=0.55),
            aero=AeroConfigBlock(Cd=0.40, Cl_front=-1.5, Cl_rear=-2.5, frontal_area_m2=1.95),
            powertrain=PowertrainConfigBlock(
                architecture="parallel", peak_power_kw=500,
                hybrid_battery_kwh=18.0, motor_peak_kw=150,
                differential="torque_vectoring",
            ),
            controls=ControlsConfigBlock(drive_mode="race", strategy_enabled=True),
        ),
    )


def electric_suv() -> VehicleDefinition:
    return VehicleDefinition(
        name="electric_suv",
        vehicle_class="ev",
        description="Battery-electric SUV",
        geometry=GeometryConfig(wheelbase_m=3.00, a_m=1.45, b_m=1.55, h_cg_m=0.65, track_front_m=1.65),
        mass=MassProperties(mass_kg=2200.0, Iz_kgm2=4200.0, fuel_mass_kg=0.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="dugoff", mu=0.95, radius_m=0.35),
            suspension=SuspensionConfig(ks_front=45000, ks_rear=50000),
            brakes=BrakeConfig(max_torque_Nm=3500, bias_front=0.62),
            aero=AeroConfigBlock(Cd=0.28, Cl_front=-0.1, Cl_rear=-0.15, frontal_area_m2=2.6),
            powertrain=PowertrainConfigBlock(
                architecture="ev", peak_power_kw=250, motor_peak_kw=250,
                hybrid_battery_kwh=90.0, gearbox="single",
            ),
            controls=ControlsConfigBlock(drive_mode="eco"),
        ),
    )


def rally_car() -> VehicleDefinition:
    return VehicleDefinition(
        name="rally_car",
        vehicle_class="rally",
        description="All-surface rally prototype",
        geometry=GeometryConfig(wheelbase_m=2.55, a_m=1.20, b_m=1.35, h_cg_m=0.48, track_front_m=1.58),
        mass=MassProperties(mass_kg=1230.0, Iz_kgm2=1900.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="dugoff", mu=0.85),
            suspension=SuspensionConfig(ks_front=50000, ks_rear=55000, motion_ratio_f=0.9),
            brakes=BrakeConfig(max_torque_Nm=3200),
            aero=AeroConfigBlock(Cd=0.45, Cl_front=-0.3, Cl_rear=-0.5),
            powertrain=PowertrainConfigBlock(peak_power_kw=280, differential="torsen"),
            controls=ControlsConfigBlock(drive_mode="sport"),
        ),
    )


def kart() -> VehicleDefinition:
    return VehicleDefinition(
        name="kart",
        vehicle_class="kart",
        description="Sprint kart",
        geometry=GeometryConfig(
            wheelbase_m=1.05, a_m=0.50, b_m=0.55, track_front_m=1.10, track_rear_m=1.20,
            h_cg_m=0.18, wheel_radius_m=0.14, overall_length_m=1.8, overall_width_m=1.3,
        ),
        mass=MassProperties(mass_kg=75.0, Iz_kgm2=15.0, fuel_mass_kg=3.0),
        subsystems=SubsystemBundle(
            tire=TireConfig(model="dugoff", mu=1.3, radius_m=0.14, Cx=20000, Cy=18000),
            suspension=SuspensionConfig(ks_front=0, ks_rear=0),  # solid axle approx
            brakes=BrakeConfig(max_torque_Nm=200, bias_front=0.0, abs_enabled=False),
            aero=AeroConfigBlock(enabled=False, Cd=0.8, frontal_area_m2=0.6),
            powertrain=PowertrainConfigBlock(peak_power_kw=15, redline_rpm=14000, gearbox="single"),
            controls=ControlsConfigBlock(abs=False, tc=False, esc=False, strategy_enabled=False),
        ),
    )


PRESETS: dict[str, callable] = {
    "generic_sedan": generic_sedan,
    "formula_sae": formula_sae,
    "gt3": gt3,
    "hypercar": hypercar,
    "electric_suv": electric_suv,
    "rally_car": rally_car,
    "kart": kart,
}


def list_presets() -> list[str]:
    return list(PRESETS.keys())


def load_preset(name: str) -> VehicleDefinition:
    if name not in PRESETS:
        raise KeyError(f"Unknown preset '{name}'. Available: {list_presets()}")
    return PRESETS[name]()
