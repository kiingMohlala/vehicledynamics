"""Phase 11.3 – System verification validation (target 20/20)."""

from __future__ import annotations

import tempfile
from pathlib import Path
import time
import numpy as np

from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
from vehicle_dynamics.simulation.replay import ReplayBuffer
from vehicle_dynamics.verification.regression_database import RegressionDatabase
from vehicle_dynamics.verification.scenario_matrix import ScenarioMatrix
from vehicle_dynamics.verification.proving_ground import ProvingGround
from vehicle_dynamics.verification.consistency_checks import ConsistencyChecker
from vehicle_dynamics.verification.numerical_monitor import NumericalMonitor
from vehicle_dynamics.verification.benchmark import BenchmarkRunner
from vehicle_dynamics.verification.regression_suite import RegressionSuite
from vehicle_dynamics.verification.report import format_verification_report, write_text_report


def test_regression_baseline() -> tuple[bool, dict]:
    with tempfile.TemporaryDirectory() as td:
        suite = RegressionSuite(baseline_root=td, dt=0.02)
        names = suite.capture_baselines()
        checks = suite.check_baselines()
        ok = len(names) == 3 and all(c.ok for c in checks)
        return ok, {"names": names, "pass": sum(c.ok for c in checks)}


def test_scenario_matrix() -> tuple[bool, dict]:
    matrix = ScenarioMatrix()
    results = matrix.run_all(dt=0.02)
    ok = all(r[1] for r in results) and len(results) >= 10
    return ok, {"n": len(results), "failed": [r[0] for r in results if not r[1]]}


def test_energy_consistency() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.emergency_braking(v0=20.0, duration=3.0))
    res = sim.run(3.0)
    con = ConsistencyChecker().check(res.telemetry)
    return con.energy_ok, {"messages": con.messages, "ok": con.ok}


def test_momentum_consistency() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.straight_acceleration(3.0))
    res = sim.run(3.0)
    con = ConsistencyChecker().check(res.telemetry)
    return con.force_ok and con.torque_ok, {"force": con.force_ok, "torque": con.torque_ok}


def test_deterministic_replay() -> tuple[bool, dict]:
    def once():
        sim = Simulation(SimulationConfig(dt=0.02, seed=42))
        sim.load_scenario(ScenarioLibrary.double_lane_change(v=14.0))
        return sim.run(3.0)

    a, b = once(), once()
    ok = ReplayBuffer(a.telemetry).matches(b.telemetry, tol=1e-9)
    return ok, {"n": len(a.telemetry.samples)}


def test_numerical_stability() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.moose_test(v=15.0))
    res = sim.run(4.0)
    rep = NumericalMonitor().check(res.telemetry, 0.02)
    return rep.ok, {"messages": rep.messages, "max_vx": rep.max_abs_vx}


def test_combined_modules() -> tuple[bool, dict]:
    """Driver + controls + powertrain + aero in one run."""
    sim = Simulation(
        SimulationConfig(dt=0.02, controls_enabled=True, powertrain_enabled=True, aero_enabled=True)
    )
    sim.load_scenario(ScenarioLibrary.crosswind(5.0))
    res = sim.run(5.0)
    ok = len(res.telemetry.samples) > 50 and np.isfinite(res.state.vehicle.vx)
    return ok, {"n": len(res.telemetry.samples), "events": res.events_fired}


def test_performance_regression() -> tuple[bool, dict]:
    bench = BenchmarkRunner(warn_ms=100.0)  # generous for CI
    results = bench.run_default()
    ok = all(r.ok for r in results)
    return ok, {r.name: r.ms_per_step for r in results}


def test_memory_regression() -> tuple[bool, dict]:
    """Sample count scales with duration; no unbounded growth across resets."""
    dt = 0.02
    def run_for(duration: float):
        sim = Simulation(SimulationConfig(dt=dt))
        sim.reset(vx=0.0)
        # open-loop steps without scenario duration override
        n = int(round(duration / dt))
        for _ in range(n):
            sim.step()
        return len(sim.telemetry.samples)

    n1 = run_for(1.0)
    n2 = run_for(2.0)
    n3 = run_for(3.0)
    ratio_21 = n2 / max(n1, 1)
    ratio_32 = n3 / max(n2, 1)
    ok = (n1 == 50) and (n2 == 100) and (n3 == 150)
    ok = ok and (1.9 < ratio_21 < 2.1) and (1.4 < ratio_32 < 1.6)
    return ok, {"n1": n1, "n2": n2, "n3": n3, "ratio_21": ratio_21}


def test_csv_integrity() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.05))
    sim.load_scenario(ScenarioLibrary.straight_acceleration(1.0))
    res = sim.run(1.0)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "t.csv"
        res.export_csv(str(p))
        text = p.read_text()
        lines = text.strip().splitlines()
        ok = len(lines) > 5 and "time" in lines[0] and "vx" in lines[0]
        return ok, {"lines": len(lines), "bytes": p.stat().st_size}


def test_report_generation() -> tuple[bool, dict]:
    with tempfile.TemporaryDirectory() as td:
        suite = RegressionSuite(baseline_root=td, dt=0.02)
        suite.capture_baselines()
        payload = {
            "baselines_captured": suite.db.list_baselines(),
            "baseline_checks": suite.check_baselines(),
            "proving_ground": ProvingGround(dt=0.02).run_braking()[:1],
            "matrix": ScenarioMatrix().run_all(dt=0.02)[:2],
            "benchmarks": BenchmarkRunner().run_default()[:1],
            "all_pass": True,
        }
        report = format_verification_report(payload)
        path = write_text_report(payload, Path(td) / "report.txt")
        ok = "Verification Report" in report and path.exists() and path.stat().st_size > 50
        return ok, {"chars": len(report), "path": str(path)}


def test_statistics_validation() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.emergency_braking(v0=25.0, duration=4.0))
    res = sim.run(4.0)
    s = res.statistics
    ok = s.n_samples > 0 and s.duration > 0 and s.max_speed > 0
    return ok, {"duration": s.duration, "max_speed": s.max_speed, "n": s.n_samples}


def test_parallel_execution() -> tuple[bool, dict]:
    """Sequential multi-scenario run as stand-in for parallel safety (shared no globals)."""
    results = []
    for builder in (
        lambda: ScenarioLibrary.straight_acceleration(2.0),
        lambda: ScenarioLibrary.emergency_braking(duration=2.0),
        lambda: ScenarioLibrary.slalom(v=11.0),
    ):
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(builder())
        results.append(sim.run(2.0))
    ok = all(len(r.telemetry.samples) > 5 for r in results)
    return ok, {"n_runs": len(results)}


def test_event_consistency() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.crosswind(6.0))
    res = sim.run(6.0)
    ok = "crosswind_gust" in res.events_fired and res.state.crosswind != 0
    return ok, {"events": res.events_fired, "wind": res.state.crosswind}


def test_long_duration_run() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.05))
    sim.load_scenario(ScenarioLibrary.straight_acceleration(15.0))
    res = sim.run(15.0)
    mon = NumericalMonitor().check(res.telemetry, 0.05)
    ok = mon.ok and res.statistics.n_samples >= 280
    return ok, {"n": res.statistics.n_samples, "num_ok": mon.ok}


def test_extreme_conditions() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.split_mu_braking(v0=28.0))
    res = sim.run(4.0)
    mon = NumericalMonitor().check(res.telemetry, 0.02)
    ok = mon.ok
    return ok, {"messages": mon.messages, "vx": res.state.vehicle.vx}


def test_configuration_matrix() -> tuple[bool, dict]:
    configs = [
        SimulationConfig(dt=0.02, controls_enabled=True, powertrain_enabled=True, aero_enabled=True),
        SimulationConfig(dt=0.02, controls_enabled=False, powertrain_enabled=True, aero_enabled=False),
        SimulationConfig(dt=0.02, controls_enabled=True, powertrain_enabled=False, aero_enabled=True),
    ]
    oks = []
    for cfg in configs:
        sim = Simulation(cfg)
        sim.load_scenario(ScenarioLibrary.straight_acceleration(2.0))
        r = sim.run(2.0)
        oks.append(np.isfinite(r.state.vehicle.vx) and len(r.telemetry.samples) > 5)
    return all(oks), {"passed": sum(oks), "total": len(oks)}


def test_solver_consistency() -> tuple[bool, dict]:
    """Same scenario, two dt values produce finite, qualitatively similar stop."""
    def brake(dt):
        sim = Simulation(SimulationConfig(dt=dt))
        sim.load_scenario(ScenarioLibrary.emergency_braking(v0=20.0, duration=4.0))
        return sim.run(4.0)

    a, b = brake(0.02), brake(0.01)
    ok = a.state.vehicle.vx < 5.0 and b.state.vehicle.vx < 5.0
    return ok, {"vx_02": a.state.vehicle.vx, "vx_01": b.state.vehicle.vx}


def test_no_nan_inf() -> tuple[bool, dict]:
    sim = Simulation(SimulationConfig(dt=0.02))
    sim.load_scenario(ScenarioLibrary.figure_eight(v=11.0))
    res = sim.run(5.0)
    d = res.telemetry.to_numpy()
    ok = all(np.all(np.isfinite(v)) for v in d.values())
    return ok, {"keys": list(d.keys())}


def test_release_candidate() -> tuple[bool, dict]:
    """Aggregate gate: baselines + short proving ground + matrix sample."""
    with tempfile.TemporaryDirectory() as td:
        suite = RegressionSuite(baseline_root=td, dt=0.02)
        suite.capture_baselines()
        checks = suite.check_baselines()
        pg = ProvingGround(dt=0.02).run_braking()
        matrix_ok = all(r[1] for r in ScenarioMatrix().run_all(dt=0.02)[:5])
        bench_ok = all(b.ok for b in BenchmarkRunner(warn_ms=100.0).run_default())
        ok = all(c.ok for c in checks) and all(p.ok for p in pg) and matrix_ok and bench_ok
        return ok, {
            "baselines": sum(c.ok for c in checks),
            "pg": sum(p.ok for p in pg),
            "matrix_sample": matrix_ok,
            "bench": bench_ok,
        }


def run_phase113_validation() -> bool:
    print("=== Phase 11.3 System Verification Validation ===\n")
    tests = [
        ("regression_baseline", test_regression_baseline),
        ("scenario_matrix", test_scenario_matrix),
        ("energy_consistency", test_energy_consistency),
        ("momentum_consistency", test_momentum_consistency),
        ("deterministic_replay", test_deterministic_replay),
        ("numerical_stability", test_numerical_stability),
        ("combined_modules", test_combined_modules),
        ("performance_regression", test_performance_regression),
        ("memory_regression", test_memory_regression),
        ("csv_integrity", test_csv_integrity),
        ("report_generation", test_report_generation),
        ("statistics_validation", test_statistics_validation),
        ("parallel_execution", test_parallel_execution),
        ("event_consistency", test_event_consistency),
        ("long_duration_run", test_long_duration_run),
        ("extreme_conditions", test_extreme_conditions),
        ("configuration_matrix", test_configuration_matrix),
        ("solver_consistency", test_solver_consistency),
        ("no_nan_inf", test_no_nan_inf),
        ("release_candidate", test_release_candidate),
    ]
    all_pass = True
    for name, fn in tests:
        t0 = time.perf_counter()
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        dt = time.perf_counter() - t0
        print(f"{name:28} : {'PASS' if ok else 'FAIL'}  ({dt:.2f}s)")
        for k, v in list(diag.items())[:4]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 11.3 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase113_validation()
