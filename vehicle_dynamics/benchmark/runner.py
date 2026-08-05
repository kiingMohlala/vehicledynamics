"""
Phase 5.5 – System Integration Benchmark Runner.

Runs the full scenario catalog through FixedStepDualTrack with all
controller combinations and produces a PASS/FAIL report.
"""

from __future__ import annotations

import time
import json
import numpy as np

from .scenarios import scenario_catalog
from .metrics import compute_metrics, metrics_to_dict
from ..dual_track.fixed_step import FixedStepDualTrack
from ..dual_track.parameters import DualTrackParameters


def run_scenario(name, sim_kwargs, flags, dt=0.002, dt_out=0.02):
    mu = flags.pop("mu_wheels", None)
    model = FixedStepDualTrack(
        params=DualTrackParameters(),
        use_abs=flags.get("use_abs", True),
        enable_esc=flags.get("enable_esc", False),
        enable_tv=flags.get("enable_tv", False),
        mu_wheels=mu,
        dt=dt,
    )
    t0 = time.perf_counter()
    res = model.simulate(dt_out=dt_out, **sim_kwargs)
    cpu = time.perf_counter() - t0
    m = compute_metrics(name, res)
    return m, cpu, res


def run_benchmark(verbose: bool = True) -> dict:
    catalog = scenario_catalog()
    results = []
    all_pass = True

    if verbose:
        print("=== Phase 5.5 System Integration Benchmark ===\n")
        print(f"{'Scenario':28} {'PASS':6} {'vx_f':>7} {'peak_r':>8} {'peak_ay':>8} {'max_u':>7} {'CPU_s':>7}")
        print("-" * 80)

    for name, sim_kw, flags in catalog:
        flags = dict(flags)  # copy
        try:
            m, cpu, _ = run_scenario(name, sim_kw, flags)
            ok = m.passed
        except Exception as e:
            from .metrics import ScenarioMetrics
            m = ScenarioMetrics(
                name=name, passed=False, final_vx=0, peak_ay=0,
                peak_yaw_rate=0, rms_yaw=0, max_utilization=0,
                stopping_distance=None, finite=False, notes=str(e),
            )
            cpu = 0.0
            ok = False

        if not ok:
            all_pass = False

        entry = metrics_to_dict(m)
        entry["cpu_s"] = round(cpu, 3)
        results.append(entry)

        if verbose:
            status = "PASS" if ok else "FAIL"
            print(
                f"{name:28} {status:6} {m.final_vx:7.2f} {m.peak_yaw_rate:8.3f} "
                f"{m.peak_ay:8.2f} {m.max_utilization:7.3f} {cpu:7.3f}"
            )
            if m.notes:
                print(f"  note: {m.notes}")

    if verbose:
        print("-" * 80)
        n_pass = sum(1 for r in results if r["passed"])
        print(f"\nPassed: {n_pass}/{len(results)}")
        print("Overall:", "ALL PASSED" if all_pass else "SOME FAILED")

    report = {
        "phase": "5.5",
        "title": "System Integration & Benchmark",
        "status": "PASS" if all_pass else "FAIL",
        "n_pass": sum(1 for r in results if r["passed"]),
        "n_total": len(results),
        "scenarios": results,
    }
    return report


def save_report(report: dict, path: str = "baseline/phase5/benchmark_report.json"):
    import os
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(report, f, indent=2)
    return path


if __name__ == "__main__":
    report = run_benchmark(verbose=True)
    # try write local baseline if cwd allows
    try:
        save_report(report)
        print("\nReport saved to baseline/phase5/benchmark_report.json")
    except Exception as e:
        print(f"\n(Could not save report locally: {e})")
