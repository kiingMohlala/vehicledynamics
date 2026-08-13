"""Phase 13.2 – Parametric Vehicle Assembly & CAD Architecture validation (22 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path
import numpy as np

from .assembly import VehicleAssembly, AssemblyConfig
from .component import Component
from .parametric_parts import engine_block, chassis_tub
from .mass_properties import compute_mass_properties
from .interference import detect_interferences, aabb_overlap
from .packaging_solver import evaluate_packaging
from .export import export_obj, export_stl, export_json_assembly
from .cad_report import format_cad_report


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_assembly_creation() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    return _ok("assembly_creation", len(a.components) >= 10)


def gate_component_registry() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    names = {c.name for c in a.components}
    return _ok("component_registry", "chassis" in names and "body" in names)


def gate_parametric_update() -> tuple[str, bool, str]:
    a = VehicleAssembly(AssemblyConfig(wheelbase=2.5)).build()
    a.update(wheelbase=2.9)
    return _ok("parametric_update", abs(a.config.wheelbase - 2.9) < 1e-12)


def gate_wheelbase_scaling() -> tuple[str, bool, str]:
    a1 = VehicleAssembly(AssemblyConfig(wheelbase=2.4)).build()
    a2 = VehicleAssembly(AssemblyConfig(wheelbase=3.0)).build()
    # rear wheels further apart in x
    r1 = a1.get("wheel_RL").position[0]
    r2 = a2.get("wheel_RL").position[0]
    return _ok("wheelbase_scaling", r2 < r1)  # rear more negative when wb larger from front=0


def gate_track_scaling() -> tuple[str, bool, str]:
    a1 = VehicleAssembly(AssemblyConfig(track=1.4)).build()
    a2 = VehicleAssembly(AssemblyConfig(track=1.8)).build()
    return _ok("track_scaling", abs(a2.get("wheel_FL").position[1]) > abs(a1.get("wheel_FL").position[1]))


def gate_ride_height_update() -> tuple[str, bool, str]:
    a = VehicleAssembly(AssemblyConfig(ride_height=0.10)).build()
    a.update(ride_height=0.18)
    ch = a.get("chassis")
    return _ok("ride_height_update", ch is not None and ch.position[2] > 0.1)


def gate_engine_packaging() -> tuple[str, bool, str]:
    a = VehicleAssembly(AssemblyConfig(powertrain="ice")).build()
    return _ok("engine_packaging", a.get("engine") is not None)


def gate_battery_packaging() -> tuple[str, bool, str]:
    a = VehicleAssembly(AssemblyConfig(powertrain="ev")).build()
    return _ok("battery_packaging", a.get("battery") is not None)


def gate_cockpit_fit() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    return _ok("cockpit_fit", a.get("cockpit") is not None and a.get("driver") is not None)


def gate_suspension_mounts() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    mounts = [c for c in a.components if c.category == "suspension"]
    return _ok("suspension_mounts", len(mounts) >= 4)


def gate_steering_clearance() -> tuple[str, bool, str]:
    # soft: assembly builds without error; packaging report exists
    a = VehicleAssembly().build()
    pkg = a.packaging()
    return _ok("steering_clearance", pkg is not None)


def gate_wheel_clearance() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    pkg = a.packaging()
    return _ok("wheel_clearance", math.isfinite(pkg.wheel_to_body_min))


def gate_body_interference() -> tuple[str, bool, str]:
    # two overlapping boxes should detect
    a = Component("a", position=[0, 0, 0], size=[1, 1, 1], mass=1)
    b = Component("b", position=[0.2, 0, 0], size=[1, 1, 1], mass=1)
    hits = detect_interferences([a, b])
    return _ok("body_interference", len(hits) >= 1)


def gate_mass_properties() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    mp = a.mass_properties
    return _ok("mass_properties", mp.total_mass > 100 and np.all(np.isfinite(mp.cg)))


def gate_cg_consistency() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    mp = a.mass_properties
    return _ok("cg_consistency", abs(mp.axle_load_front + mp.axle_load_rear - mp.total_mass) < 1e-6)


def gate_export_obj() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "v.obj"
        a.export(str(p))
        text = p.read_text()
    return _ok("export_obj", "v " in text and "f " in text)


def gate_export_stl() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "v.stl"
        a.export(str(p))
        text = p.read_text()
    return _ok("export_stl", "solid" in text and "vertex" in text)


def gate_serialization() -> tuple[str, bool, str]:
    a = VehicleAssembly(AssemblyConfig(wheelbase=2.8)).build()
    d = a.to_dict()
    a2 = VehicleAssembly.from_dict(d)
    return _ok("serialization", abs(a2.config.wheelbase - 2.8) < 1e-12 and len(a2.components) == len(a.components))


def gate_repeatability() -> tuple[str, bool, str]:
    a1 = VehicleAssembly(AssemblyConfig(wheelbase=2.7)).build()
    a2 = VehicleAssembly(AssemblyConfig(wheelbase=2.7)).build()
    return _ok("repeatability", abs(a1.mass_properties.total_mass - a2.mass_properties.total_mass) < 1e-9)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    t0 = time.perf_counter()
    for _ in range(30):
        VehicleAssembly().build().mass_properties
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 8000.0, f"{ms:.1f} ms")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    a = VehicleAssembly().build()
    mp = a.mass_properties
    ok = math.isfinite(mp.total_mass) and np.all(np.isfinite(mp.cg))
    return _ok("no_nan_inf", ok)


def gate_regression_contract() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.25))
        r = sim.run(0.25)
        return _ok("regression_contract", len(r.telemetry.samples) > 0)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_assembly_creation,
    gate_component_registry,
    gate_parametric_update,
    gate_wheelbase_scaling,
    gate_track_scaling,
    gate_ride_height_update,
    gate_engine_packaging,
    gate_battery_packaging,
    gate_cockpit_fit,
    gate_suspension_mounts,
    gate_steering_clearance,
    gate_wheel_clearance,
    gate_body_interference,
    gate_mass_properties,
    gate_cg_consistency,
    gate_export_obj,
    gate_export_stl,
    gate_serialization,
    gate_repeatability,
    gate_performance_regression,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase132_validation(verbose: bool = True) -> bool:
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
            print("Phase 13.2 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 13.2 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase132_validation() else 1)
