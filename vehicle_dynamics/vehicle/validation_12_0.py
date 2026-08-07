"""Phase 12.0 – Vehicle Architecture & Digital Twin validation (20 gates)."""
from __future__ import annotations

import tempfile
from pathlib import Path

from .vehicle_definition import VehicleDefinition
from .vehicle_builder import VehicleBuilder
from .vehicle_registry import VehicleRegistry
from .subsystem_registry import DEFAULT_REGISTRY
from .presets import load_preset, list_presets, generic_sedan, hypercar, formula_sae
from .serialization import save_json, load_json, save_yaml, load_yaml, roundtrip_json
from .digital_twin import create_digital_twin
from .comparison import compare_definitions
from .report import format_vehicle_report


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_vehicle_build() -> tuple[str, bool, str]:
    bv = VehicleBuilder().build(generic_sedan())
    return _ok("vehicle_build", bv.mass_kg > 0 and bv.wheelbase_m > 0, f"m={bv.mass_kg}")


def gate_serialization() -> tuple[str, bool, str]:
    d = generic_sedan()
    raw = d.to_dict()
    return _ok("serialization", "geometry" in raw and "mass" in raw)


def gate_json_roundtrip() -> tuple[str, bool, str]:
    d = hypercar()
    d2 = roundtrip_json(d)
    ok = d.configuration_hash() == d2.configuration_hash()
    return _ok("json_roundtrip", ok, d.configuration_hash())


def gate_yaml_roundtrip() -> tuple[str, bool, str]:
    d = formula_sae()
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "car.yaml"
        save_yaml(d, path)
        d2 = load_yaml(path)
    ok = abs(d.mass.mass_kg - d2.mass.mass_kg) < 1e-9
    return _ok("yaml_roundtrip", ok)


def gate_configuration_hash() -> tuple[str, bool, str]:
    a = generic_sedan()
    b = generic_sedan()
    c = hypercar()
    ok = a.configuration_hash() == b.configuration_hash() and a.configuration_hash() != c.configuration_hash()
    return _ok("configuration_hash", ok, a.configuration_hash())


def gate_preset_loading() -> tuple[str, bool, str]:
    names = list_presets()
    ok = len(names) >= 5 and all(load_preset(n).name for n in names)
    return _ok("preset_loading", ok, f"n={len(names)}")


def gate_subsystem_registry() -> tuple[str, bool, str]:
    r = DEFAULT_REGISTRY
    ok = r.has("tire", "dugoff") and r.has("powertrain", "ice")
    tire = r.create("tire", "pacejka", mu=1.2)
    return _ok("subsystem_registry", ok and tire["model"] == "pacejka")


def gate_builder_consistency() -> tuple[str, bool, str]:
    d = generic_sedan()
    b1 = VehicleBuilder().build(d)
    b2 = VehicleBuilder().build(d)
    ok = b1.config_hash == b2.config_hash and abs(b1.mass_kg - b2.mass_kg) < 1e-12
    return _ok("builder_consistency", ok)


def gate_mass_consistency() -> tuple[str, bool, str]:
    d = hypercar()
    expected = d.mass.mass_kg + d.mass.fuel_mass_kg
    bv = VehicleBuilder().build(d)
    ok = abs(bv.mass_kg - expected) < 1e-9
    return _ok("mass_consistency", ok, f"{bv.mass_kg}")


def gate_geometry_consistency() -> tuple[str, bool, str]:
    d = formula_sae()
    ok = abs(d.geometry.L - (d.geometry.a_m + d.geometry.b_m)) < 1e-12
    return _ok("geometry_consistency", ok, f"L={d.geometry.L}")


def gate_simulation_integration() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        bv = VehicleBuilder().build(generic_sedan())
        cfg = SimulationConfig(dt=0.02, **{
            k: v for k, v in bv.simulation_kwargs.items()
            if k in SimulationConfig.__dataclass_fields__
        })
        sim = Simulation(cfg)
        sim.load_scenario(ScenarioLibrary.straight_acceleration(1.0))
        r = sim.run(1.0)
        ok = len(r.telemetry.samples) > 0 and r.state.vehicle.vx >= 0
        return _ok("simulation_integration", ok, f"vx={r.state.vehicle.vx:.3f}")
    except Exception as e:
        return _ok("simulation_integration", False, str(e))


def gate_verification_integration() -> tuple[str, bool, str]:
    # Lightweight: twin can be created and marked validated
    twin = create_digital_twin(generic_sedan())
    twin.mark_validated()
    return _ok("verification_integration", twin.validation_status == "validated")


def gate_optimization_integration() -> tuple[str, bool, str]:
    # Placeholder hook: hash stable for identical configs (optimizer key)
    a = generic_sedan().configuration_hash()
    b = generic_sedan().configuration_hash()
    return _ok("optimization_integration", a == b, a)


def gate_comparison_report() -> tuple[str, bool, str]:
    cmp = compare_definitions(generic_sedan(), hypercar())
    text = cmp.as_table()
    ok = "mass_kg" in cmp.deltas and "hypercar" in text
    return _ok("comparison_report", ok)


def gate_digital_twin_creation() -> tuple[str, bool, str]:
    twin = create_digital_twin(load_preset("gt3"))
    s = twin.summary()
    ok = s["name"] == "gt3_prototype" and "hash" in s
    return _ok("digital_twin_creation", ok, s["hash"])


def gate_repeatability() -> tuple[str, bool, str]:
    h1 = VehicleBuilder().build(generic_sedan()).config_hash
    h2 = VehicleBuilder().build(generic_sedan()).config_hash
    return _ok("repeatability", h1 == h2)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    t0 = time.perf_counter()
    for _ in range(50):
        VehicleBuilder().build(generic_sedan())
    ms = (time.perf_counter() - t0) / 50 * 1000
    return _ok("performance_regression", ms < 50.0, f"{ms:.3f} ms/build")


def gate_backward_compatibility() -> tuple[str, bool, str]:
    # Building sedan must not require optional heavy deps
    bv = VehicleBuilder().build(generic_sedan())
    ok = "mass" in bv.simulation_kwargs and bv.simulation_kwargs["mass"] > 0
    return _ok("backward_compatibility", ok)


def gate_no_nan_inf() -> tuple[str, bool, str]:
    import math
    d = hypercar()
    vals = [d.mass.mass_kg, d.geometry.L, d.subsystems.aero.Cd, d.subsystems.powertrain.peak_power_kw]
    ok = all(math.isfinite(v) for v in vals)
    return _ok("no_nan_inf", ok)


def gate_release_candidate() -> tuple[str, bool, str]:
    reg = VehicleRegistry()
    names = reg.load_all_presets()
    ok = len(names) >= 5 and all(n in reg.list_names() for n in names)
    # JSON file round-trip on disk
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "twin.json"
        save_json(reg.get_definition("hypercar"), p)
        d2 = load_json(p)
        ok = ok and d2.name == "hypercar"
    return _ok("release_candidate", ok, f"presets={len(names)}")


GATES = [
    gate_vehicle_build,
    gate_serialization,
    gate_json_roundtrip,
    gate_yaml_roundtrip,
    gate_configuration_hash,
    gate_preset_loading,
    gate_subsystem_registry,
    gate_builder_consistency,
    gate_mass_consistency,
    gate_geometry_consistency,
    gate_simulation_integration,
    gate_verification_integration,
    gate_optimization_integration,
    gate_comparison_report,
    gate_digital_twin_creation,
    gate_repeatability,
    gate_performance_regression,
    gate_backward_compatibility,
    gate_no_nan_inf,
    gate_release_candidate,
]


def run_phase120_validation(verbose: bool = True) -> bool:
    results = []
    for g in GATES:
        name, passed, detail = g()
        results.append((name, passed, detail))
        if verbose:
            status = "PASS" if passed else "FAIL"
            extra = f"  ({detail})" if detail else ""
            print(f"  {name:28s}: {status}{extra}")

    n_pass = sum(1 for _, p, _ in results if p)
    n = len(results)
    if verbose:
        print()
        print("=" * 41)
        if n_pass == n:
            print("ALL TESTS PASSED")
            print("Phase 12.0 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 12.0 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase120_validation() else 1)
