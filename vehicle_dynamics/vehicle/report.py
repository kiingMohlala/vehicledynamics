"""Engineering report for a vehicle definition / digital twin."""
from __future__ import annotations

from .vehicle_definition import VehicleDefinition
from .digital_twin import DigitalTwin


def format_vehicle_report(defn: VehicleDefinition, twin: DigitalTwin | None = None) -> str:
    g = defn.geometry
    m = defn.mass
    s = defn.subsystems
    lines = [
        f"Vehicle Report: {defn.name}",
        "=" * 50,
        f"Class            : {defn.vehicle_class}",
        f"Version          : {defn.version}",
        f"Description      : {defn.description or '(none)'}",
        f"Config hash      : {defn.configuration_hash()}",
        "",
        "Geometry",
        "-" * 30,
        f"  Wheelbase      : {g.L:.3f} m",
        f"  Track F/R      : {g.track_front_m:.3f} / {g.track_rear_m:.3f} m",
        f"  CG a/b/h       : {g.a_m:.3f} / {g.b_m:.3f} / {g.h_cg_m:.3f} m",
        f"  Wheel radius   : {g.wheel_radius_m:.3f} m",
        "",
        "Mass",
        "-" * 30,
        f"  Curb mass      : {m.mass_kg:.1f} kg",
        f"  Fuel mass      : {m.fuel_mass_kg:.1f} kg",
        f"  Total          : {m.total_mass_kg:.1f} kg",
        f"  Iz             : {m.Iz_kgm2:.1f} kg·m²",
        "",
        "Powertrain",
        "-" * 30,
        f"  Architecture   : {s.powertrain.architecture}",
        f"  Peak power     : {s.powertrain.peak_power_kw:.0f} kW",
        f"  Differential   : {s.powertrain.differential}",
        f"  Battery        : {s.powertrain.hybrid_battery_kwh:.1f} kWh",
        "",
        "Chassis / Aero / Tires",
        "-" * 30,
        f"  Tire model     : {s.tire.model}  μ={s.tire.mu}",
        f"  Cd / Cl_f/r    : {s.aero.Cd} / {s.aero.Cl_front} / {s.aero.Cl_rear}",
        f"  Brake torque   : {s.brakes.max_torque_Nm:.0f} Nm  bias={s.brakes.bias_front}",
        f"  Controls       : ABS={s.controls.abs} TC={s.controls.tc} ESC={s.controls.esc}",
        f"  Drive mode     : {s.controls.drive_mode}",
    ]
    if twin is not None:
        lines += [
            "",
            "Digital Twin",
            "-" * 30,
            f"  Status         : {twin.validation_status}",
            f"  Created        : {twin.created_at}",
            f"  Hash           : {twin.config_hash}",
        ]
    return "\n".join(lines)
