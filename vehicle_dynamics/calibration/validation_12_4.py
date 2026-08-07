"""Phase 12.4 – Model Calibration validation (20 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np

from .telemetry_loader import load_telemetry, synthesize_telemetry, TelemetryData
from .signal_processing import process_telemetry, butterworth_lowpass_simple, estimate_noise_std, fill_nan
from .synchronization import lag_by_correlation, align_signals
from .parameter_sets import ParameterSet, CalibParameter
from .objective_functions import signal_cost
from .optimizer import nelder_mead, differential_evolution, least_squares, grid_search
from .parameter_identification import coastdown_vx, tire_force_curve, suspension_step_response, make_coastdown_model
from .validation_metrics import rmse, summary_metrics, r2_score
from .uncertainty import bootstrap_uncertainty
from .calibration_database import CalibrationDatabase
from .calibration_report import format_calibration_report, export_calibration_report
from .calibration_runner import CalibrationRunner, calibrate_tire_curve, calibrate_suspension_step


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_telemetry_import() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "log.csv"
        p.write_text("time,vx,ax\n0,20,0\n0.1,19.8,-2\n0.2,19.5,-3\n")
        data = load_telemetry(p)
    return _ok("telemetry_import", data.n == 3 and "vx" in data.channels)


def gate_signal_filtering() -> tuple[str, bool, str]:
    y = np.array([0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0])
    f = butterworth_lowpass_simple(y, alpha=0.3)
    return _ok("signal_filtering", len(f) == len(y) and np.all(np.isfinite(f)))


def gate_time_alignment() -> tuple[str, bool, str]:
    t = np.linspace(0, 1, 100)
    a = np.sin(2 * np.pi * 2 * t)
    b = np.sin(2 * np.pi * 2 * (t - 0.05))
    lag = lag_by_correlation(a, b, max_lag=20)
    return _ok("time_alignment", abs(lag) >= 1, f"lag={lag}")


def gate_parameter_loading() -> tuple[str, bool, str]:
    ps = ParameterSet.default_vehicle()
    return _ok("parameter_loading", len(ps.params) >= 5 and "Cd" in ps.values())


def gate_objective_evaluation() -> tuple[str, bool, str]:
    a = np.array([1.0, 2.0, 3.0])
    b = np.array([1.1, 2.1, 2.9])
    c = signal_cost(a, b, metric="rmse")
    return _ok("objective_evaluation", c > 0 and math.isfinite(c), f"{c:.4f}")


def gate_least_squares_optimizer() -> tuple[str, bool, str]:
    def residual(x):
        return np.array([x[0] - 2.0, x[1] - 3.0])
    r = least_squares(residual, np.array([0.0, 0.0]))
    return _ok("least_squares_optimizer", abs(r.x[0] - 2) < 0.2 and abs(r.x[1] - 3) < 0.2, f"x={r.x}")


def gate_differential_evolution() -> tuple[str, bool, str]:
    def fun(x):
        return float((x[0] - 1.5) ** 2 + (x[1] + 0.5) ** 2)
    r = differential_evolution(fun, [(-2, 2), (-2, 2)], maxiter=15, seed=0)
    return _ok("differential_evolution", r.fun < 0.05, f"f={r.fun:.4f}")


def gate_parameter_identification() -> tuple[str, bool, str]:
    t = np.linspace(0, 5, 200)
    true = coastdown_vx(t, 25.0, mass=1400, Cd=0.35, rr=0.012)
    model = make_coastdown_model(t, 25.0)
    def cost(x):
        return rmse(true, model({"mass": x[0], "Cd": x[1], "rolling_resistance": x[2]}))
    r = nelder_mead(cost, np.array([1500.0, 0.4, 0.02]), maxiter=80)
    return _ok("parameter_identification", r.fun < 0.5, f"rmse={r.fun:.3f}")


def gate_tire_calibration() -> tuple[str, bool, str]:
    slip = np.linspace(-0.2, 0.2, 41)
    force = tire_force_curve(slip, mu=1.1, Cx=90000.0)
    force = force + np.random.default_rng(0).normal(0, 30, size=force.shape)
    res = calibrate_tire_curve(slip, force)
    return _ok("tire_calibration", abs(res["tire_mu"] - 1.1) < 0.25, f"mu={res['tire_mu']:.3f}")


def gate_suspension_calibration() -> tuple[str, bool, str]:
    t = np.linspace(0, 2, 200)
    z = suspension_step_response(t, k=28000, c=2200, m=300, z0=0.04)
    res = calibrate_suspension_step(t, z, m=300)
    return _ok("suspension_calibration", res["rmse"] < 0.01, f"rmse={res['rmse']:.4f}")


def gate_aero_calibration() -> tuple[str, bool, str]:
    tel = synthesize_telemetry(duration=6.0, v0=30.0, ax=-1.2, noise=0.02, seed=1)
    runner = CalibrationRunner(method="nelder-mead")
    result = runner.calibrate(tel, parameters=["Cd", "rolling_resistance", "mass"], signal="vx")
    return _ok("aero_calibration", result.rmse < 1.5, f"rmse={result.rmse:.3f}")


def gate_validation_metrics() -> tuple[str, bool, str]:
    a = np.linspace(0, 1, 50)
    b = a + 0.01
    m = summary_metrics(a, b)
    return _ok("validation_metrics", m["rmse"] < 0.02 and m["r2"] > 0.99)


def gate_uncertainty_estimation() -> tuple[str, bool, str]:
    def fun(x):
        return float((x[0] - 1) ** 2)
    unc = bootstrap_uncertainty(fun, np.array([1.0]), ["p"], [(0.0, 2.0)], n_boot=10)
    return _ok("uncertainty_estimation", 0 <= unc.confidence_score <= 1, f"c={unc.confidence_score:.3f}")


def gate_database_storage() -> tuple[str, bool, str]:
    db = CalibrationDatabase()
    db.add({"Cd": 0.3}, {"rmse": 0.1}, method="test")
    return _ok("database_storage", len(db) == 1)


def gate_report_generation() -> tuple[str, bool, str]:
    res = {"method": "test", "rmse": 0.1, "r2": 0.99, "nfev": 10, "confidence": 0.8,
           "best_parameters": {"Cd": 0.33}, "initial_parameters": {"Cd": 0.4}}
    text = format_calibration_report(res)
    with tempfile.TemporaryDirectory() as td:
        export_calibration_report(res, Path(td) / "c.md")
        ok = "Best parameters" in text and (Path(td) / "c.md").exists()
    return _ok("report_generation", ok)


def gate_repeatability() -> tuple[str, bool, str]:
    tel = synthesize_telemetry(duration=3.0, seed=42, noise=0.0)
    r1 = CalibrationRunner(method="nelder-mead").calibrate(tel, parameters=["Cd", "rolling_resistance"])
    r2 = CalibrationRunner(method="nelder-mead").calibrate(tel, parameters=["Cd", "rolling_resistance"])
    ok = abs(r1.rmse - r2.rmse) < 1e-6
    return _ok("repeatability", ok, f"{r1.rmse:.6f}")


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    tel = synthesize_telemetry(duration=2.0, seed=0, noise=0.01)
    t0 = time.perf_counter()
    CalibrationRunner(method="nelder-mead").calibrate(tel, parameters=["Cd", "rolling_resistance"])
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 15000.0, f"{ms:.0f} ms")


def gate_large_dataset() -> tuple[str, bool, str]:
    tel = synthesize_telemetry(duration=20.0, dt=0.01, seed=2, noise=0.02)
    data = process_telemetry(tel)
    return _ok("large_dataset", data.n > 1500, f"n={data.n}")


def gate_no_nan_inf() -> tuple[str, bool, str]:
    y = np.array([1.0, np.nan, 3.0, np.inf, 5.0])
    f = fill_nan(y)
    return _ok("no_nan_inf", np.all(np.isfinite(f)))


def gate_regression_contract() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.3))
        r = sim.run(0.3)
        return _ok("regression_contract", len(r.telemetry.samples) > 0)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_telemetry_import,
    gate_signal_filtering,
    gate_time_alignment,
    gate_parameter_loading,
    gate_objective_evaluation,
    gate_least_squares_optimizer,
    gate_differential_evolution,
    gate_parameter_identification,
    gate_tire_calibration,
    gate_suspension_calibration,
    gate_aero_calibration,
    gate_validation_metrics,
    gate_uncertainty_estimation,
    gate_database_storage,
    gate_report_generation,
    gate_repeatability,
    gate_performance_regression,
    gate_large_dataset,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase124_validation(verbose: bool = True) -> bool:
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
            print("Phase 12.4 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 12.4 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase124_validation() else 1)
