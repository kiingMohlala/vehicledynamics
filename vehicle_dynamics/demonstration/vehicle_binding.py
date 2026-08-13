"""
Phase 14.2H — Authoritative vehicle binding.

Two explicit identities (never mixed):

  HISTORICAL_DEMONSTRATOR  1400 kg / 280 kW   — frozen 14.2D/E artifact
  AUTHORITATIVE_HYPERCAR   1100 kg / 750 kW   — project reference vehicle

All runtime simulation parameters for the hypercar must flow from
ReferenceBuild / VehicleDefinition — not from ad-hoc frozen_cfg() overrides.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from typing import Any, Optional

from vehicle_dynamics.simulation.simulation import SimulationConfig
from vehicle_dynamics.demonstration.reality_reference import (
    ReferenceBuild,
    build_hypercar_demonstrator,
)


# ---------------------------------------------------------------------------
# Historical 14.2D/E demonstrator (immutable regression artifact)
# ---------------------------------------------------------------------------
HISTORICAL_DEMONSTRATOR_ID = "historical_14_2de_demonstrator_1400_280"


def historical_demonstrator_config() -> SimulationConfig:
    """Exact plant used for 14.2D/E metrics (5.36 s / 19.77 s). Do not mutate."""
    return SimulationConfig(
        dt=0.01,
        mass=1400.0,
        Iz=2500.0,
        wheelbase=2.7,
        track=1.55,
        wheel_radius=0.32,
        peak_torque_nm=450.0,
        peak_power_kw=280.0,
        peak_torque_rpm=4500.0,
        redline_rpm=7500.0,
        final_drive=3.9,
        mu_tire=1.15,
        use_dual_track=True,
        abs_enabled=True,
        drive_split_front=0.35,
        aero_enabled=True,
        seed=0,
    )


HISTORICAL_EXPECTED = {
    "t100_s": 5.36,
    "t200_s": 19.77,
    "t_brake_100_0_s": 2.30,
    "mass_kg": 1400.0,
    "peak_power_kw": 280.0,
}


# ---------------------------------------------------------------------------
# Authoritative hypercar binding
# ---------------------------------------------------------------------------
AUTHORITATIVE_HYPERCAR_ID = "authoritative_hypercar_1100_750"


def _fingerprint(payload: dict[str, Any]) -> str:
    raw = json.dumps(payload, sort_keys=True, default=str)
    return hashlib.sha1(raw.encode()).hexdigest()[:16]


@dataclass
class BoundVehicle:
    """Runtime-bound vehicle with full provenance."""

    identity: str
    mass_kg: float
    peak_power_kw: float
    drivetrain: str
    aero_mode: str
    simulation_config: SimulationConfig
    definition_meta: dict[str, Any] = field(default_factory=dict)
    provenance: dict[str, str] = field(default_factory=dict)
    config_fingerprint: str = ""
    twin_hash: str = ""

    def runtime_identity_ok(self) -> tuple[bool, str]:
        """Inspect actual SimulationConfig — not the brief."""
        c = self.simulation_config
        checks = [
            (abs(c.mass - 1100.0) < 1.0, f"mass={c.mass} want 1100"),
            (abs(c.peak_power_kw - 750.0) < 1.0, f"power={c.peak_power_kw} want 750"),
            (c.drive_split_front > 0.0, f"drive_split={c.drive_split_front} want AWD>0"),
            (c.aero_enabled is True, f"aero_enabled={c.aero_enabled}"),
            (c.use_dual_track is True, f"dual_track={c.use_dual_track}"),
        ]
        fails = [msg for ok, msg in checks if not ok]
        return (len(fails) == 0, "; ".join(fails) if fails else "ok")


def bind_authoritative_hypercar(
    mass_kg: float = 1100.0,
    power_kw: float = 750.0,
) -> BoundVehicle:
    """
    Authoritative path:
      VehicleDefinition (1100/750)
        → DigitalTwin
        → SimulationConfig (fully populated)
    """
    # fuel_mass_kg=0 so total mass equals brief curb mass (identity gate)
    from vehicle_dynamics.vehicle.geometry import GeometryConfig
    from vehicle_dynamics.vehicle.mass_properties import MassProperties
    from vehicle_dynamics.vehicle.configuration import (
        SubsystemBundle,
        PowertrainConfigBlock,
        AeroConfigBlock,
        SuspensionConfig,
        TireConfig,
        BrakeConfig,
    )
    from vehicle_dynamics.vehicle import VehicleDefinition, create_digital_twin

    definition = VehicleDefinition(
        name="Hypercar_Demonstrator_Authoritative",
        vehicle_class="hypercar",
        version="14.2H",
        description="Phase 14.2H authoritative: 750 kW AWD 1100 kg high-downforce",
        geometry=GeometryConfig(
            wheelbase_m=2.70,
            track_front_m=1.65,
            track_rear_m=1.62,
            a_m=1.25,
            b_m=1.45,
            h_cg_m=0.40,
            wheel_radius_m=0.33,
        ),
        mass=MassProperties(
            mass_kg=mass_kg,
            fuel_mass_kg=0.0,  # identity: total == brief mass
            Iz_kgm2=2200.0,
            Ix_kgm2=500.0,
            Iy_kgm2=1800.0,
        ),
        subsystems=SubsystemBundle(
            tire=TireConfig(
                model="dugoff",
                mu=1.15,
                Cx=100000.0,
                Cy=90000.0,
                radius_m=0.33,
            ),
            suspension=SuspensionConfig(),
            brakes=BrakeConfig(max_torque_Nm=2800.0, bias_front=0.58, abs_enabled=True),
            aero=AeroConfigBlock(
                enabled=True,
                Cd=0.34,
                Cl_front=-0.55,  # high_downforce package (still from aero block, not retuned for time)
                Cl_rear=-0.85,
                frontal_area_m2=1.90,
            ),
            powertrain=PowertrainConfigBlock(
                architecture="awd",
                peak_power_kw=power_kw,
                redline_rpm=8500.0,
                idle_rpm=900.0,
                final_drive=3.9,
                gearbox="dct",
                differential="open",
            ),
        ),
        metadata={
            "phase": "14.2H",
            "frozen": True,
            "identity": AUTHORITATIVE_HYPERCAR_ID,
            "drivetrain": "awd",
            "aero": "high_downforce",
            "suspension": "double_wishbone",
            "tires": "performance_road_track",
        },
    )
    twin = create_digital_twin(definition)
    pt = definition.subsystems.powertrain
    tire = definition.subsystems.tire
    brakes = definition.subsystems.brakes
    geo = definition.geometry

    # peak torque from power at 4500 rpm map point (same derivation as Simulation)
    omega_pt = 4500.0 * 2.0 * 3.141592653589793 / 60.0
    peak_tq = (power_kw * 1000.0) / omega_pt

    sim_cfg = SimulationConfig(
        dt=0.01,
        mass=float(definition.mass.total_mass_kg),
        Iz=float(definition.mass.Iz_kgm2),
        wheelbase=float(geo.wheelbase_m),
        track=float(0.5 * (geo.track_front_m + geo.track_rear_m)),
        wheel_radius=float(tire.radius_m),
        CdA=float(definition.subsystems.aero.Cd * definition.subsystems.aero.frontal_area_m2),
        controls_enabled=True,
        powertrain_enabled=True,
        strategy_enabled=True,
        drive_mode="sport",
        aero_enabled=bool(definition.subsystems.aero.enabled),
        seed=42,
        peak_torque_nm=float(peak_tq),
        peak_torque_rpm=4500.0,
        peak_power_kw=float(pt.peak_power_kw),
        redline_rpm=float(pt.redline_rpm),
        idle_rpm=float(pt.idle_rpm),
        final_drive=float(pt.final_drive),
        mu_tire=float(tire.mu),
        use_dual_track=True,
        abs_enabled=bool(brakes.abs_enabled),
        drive_split_front=0.35,  # AWD — explicit; no silent RWD
    )

    provenance = {
        "mass": "VehicleDefinition.mass.total_mass_kg → SimulationConfig.mass",
        "peak_power_kw": "PowertrainConfigBlock.peak_power_kw → SimulationConfig.peak_power_kw",
        "peak_torque_nm": "derived P/ω@4500rpm → SimulationConfig.peak_torque_nm",
        "mu_tire": "TireConfig.mu → SimulationConfig.mu_tire",
        "wheel_radius": "TireConfig.radius_m → SimulationConfig.wheel_radius",
        "final_drive": "PowertrainConfigBlock.final_drive → SimulationConfig.final_drive",
        "wheelbase": "GeometryConfig.wheelbase_m → SimulationConfig.wheelbase",
        "track": "mean(track_f,track_r) → SimulationConfig.track",
        "Iz": "MassProperties.Iz_kgm2 → SimulationConfig.Iz",
        "abs_enabled": "BrakeConfig.abs_enabled → SimulationConfig.abs_enabled",
        "aero_enabled": "AeroConfigBlock.enabled → SimulationConfig.aero_enabled",
        "drive_split_front": "AWD policy 0.35 → SimulationConfig.drive_split_front",
        "use_dual_track": "authoritative plant policy → True",
    }

    fp_payload = {
        "identity": AUTHORITATIVE_HYPERCAR_ID,
        "mass": sim_cfg.mass,
        "power": sim_cfg.peak_power_kw,
        "mu": sim_cfg.mu_tire,
        "r": sim_cfg.wheel_radius,
        "fd": sim_cfg.final_drive,
        "split": sim_cfg.drive_split_front,
        "wb": sim_cfg.wheelbase,
        "Iz": sim_cfg.Iz,
        "redline": sim_cfg.redline_rpm,
    }
    return BoundVehicle(
        identity=AUTHORITATIVE_HYPERCAR_ID,
        mass_kg=float(sim_cfg.mass),
        peak_power_kw=float(sim_cfg.peak_power_kw),
        drivetrain="awd",
        aero_mode="high_downforce",
        simulation_config=sim_cfg,
        definition_meta=dict(definition.metadata),
        provenance=provenance,
        config_fingerprint=_fingerprint(fp_payload),
        twin_hash=str(twin.config_hash),
    )


def bind_historical_demonstrator() -> BoundVehicle:
    cfg = historical_demonstrator_config()
    fp = _fingerprint({
        "identity": HISTORICAL_DEMONSTRATOR_ID,
        "mass": cfg.mass,
        "power": cfg.peak_power_kw,
        "mu": cfg.mu_tire,
        "r": cfg.wheel_radius,
    })
    return BoundVehicle(
        identity=HISTORICAL_DEMONSTRATOR_ID,
        mass_kg=cfg.mass,
        peak_power_kw=cfg.peak_power_kw,
        drivetrain="awd",
        aero_mode="enabled",
        simulation_config=cfg,
        definition_meta={"phase": "14.2D/E", "historical": True},
        provenance={"all": "historical_demonstrator_config() frozen artifact"},
        config_fingerprint=fp,
    )
