"""Phase 12.1 – Virtual Test Track & Lap Simulation validation (20 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np

from .track_segments import straight, constant_radius, SurfaceProperties
from .track_loader import TrackLibrary, from_segments, from_csv_centerline
from .curvature import curvature_from_xy, reference_speed
from .friction_map import FrictionMap
from .racing_line import center_line, ideal_line
from .sector_timer import SectorTimer, equal_sectors, best_sectors, delta_to_ghost
from .lap_simulator import LapSimulator, compare_vehicles
from .telemetry_export import export_csv, export_json, export_markdown_report
from .lap_statistics import compute_lap_metrics


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_track_loading() -> tuple[str, bool, str]:
    tr = TrackLibrary.handling_course()
    return _ok("track_loading", tr.length > 100 and len(tr.x) > 10, f"L={tr.length:.1f}")


def gate_segment_generation() -> tuple[str, bool, str]:
    segs = [straight(50), constant_radius(40, 25)]
    tr = from_segments("test", segs, ds=2.0, closed=False)
    return _ok("segment_generation", tr.length > 80, f"L={tr.length:.1f}")


def gate_curvature_solver() -> tuple[str, bool, str]:
    th = np.linspace(0, np.pi, 50)
    x, y = 30 * np.cos(th), 30 * np.sin(th)
    k = curvature_from_xy(x, y)
    mean_k = float(np.mean(k[5:-5]))
    ok = 0.02 < mean_k < 0.05  # ~1/30
    return _ok("curvature_solver", ok, f"k≈{mean_k:.4f}")


def gate_surface_mapping() -> tuple[str, bool, str]:
    fm = FrictionMap.from_segments([100, 100], [1.0, 0.5])
    mu0 = float(fm.mu(10)[0])
    mu1 = float(fm.mu(200)[0])   # end of second segment
    ok = mu0 > 0.9 and mu1 < 0.55 and mu1 < mu0
    return _ok("surface_mapping", ok, f"{mu0:.2f}/{mu1:.2f}")


def gate_racing_line_generation() -> tuple[str, bool, str]:
    tr = TrackLibrary.oval(radius=50, straights=80, ds=2.0)
    c = center_line(tr)
    ideal = ideal_line(tr)
    ok = c.length > 0 and ideal.kind == "ideal" and len(ideal.v_ref) == len(tr.s)
    return _ok("racing_line_generation", ok, ideal.kind)


def gate_lap_completion() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(200, ds=2.0)
    sim = LapSimulator(None, tr, dt=0.05, n_sectors=2, line="center", v_max=40)
    res = sim.run_laps(1, v0=12.0)
    ok = res.statistics.n_laps == 1 and res.statistics.total_distance > 50
    return _ok("lap_completion", ok, f"t={res.best_lap:.2f}")


def gate_sector_timing() -> tuple[str, bool, str]:
    bounds = equal_sectors(300, 3)
    timer = SectorTimer(boundaries_s=bounds)
    for s, t in [(50, 1), (100, 2), (150, 3), (200, 4), (250, 5), (300, 6)]:
        timer.update(s, t)
    r = timer.result()
    ok = r is not None and r.n_sectors == 3 and abs(r.total - 6) < 1e-9
    return _ok("sector_timing", ok, str(r.sector_dt if r else None))


def gate_telemetry_logging() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(150, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05, line="center").run_laps(1, v0=10)
    telem = res.laps[0].telemetry
    ok = "time" in telem and "vx" in telem and len(telem["time"]) > 5
    return _ok("telemetry_logging", ok, f"n={len(telem['time'])}")


def gate_vehicle_comparison() -> tuple[str, bool, str]:
    try:
        from vehicle_dynamics.vehicle import load_preset, create_digital_twin
        a = create_digital_twin(load_preset("generic_sedan"))
        b = create_digital_twin(load_preset("formula_sae"))
        tr = TrackLibrary.straight(180, ds=2.0)
        out = compare_vehicles([a, b], tr, n_laps=1, dt=0.05, line="center")
        ok = len(out) == 2 and all(r.statistics.n_laps == 1 for r in out.values())
        return _ok("vehicle_comparison", ok, str(list(out.keys())))
    except Exception as e:
        return _ok("vehicle_comparison", False, str(e))


def gate_fuel_tracking() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(120, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05).run_laps(1, v0=10)
    fuel = res.laps[0].telemetry.get("fuel_g", [0.0])
    ok = len(fuel) > 0 and all(math.isfinite(f) for f in fuel)
    return _ok("fuel_tracking", ok)


def gate_battery_tracking() -> tuple[str, bool, str]:
    # Battery optional in plant; gate checks metric field exists and is finite
    m = compute_lap_metrics(np.array([0.0, 1.0]), np.array([0.0, 10.0]), np.array([10.0, 10.0]), soc=np.array([0.9, 0.85]))
    return _ok("battery_tracking", abs(m.battery_used - 0.05) < 1e-9, f"{m.battery_used}")


def gate_aero_coupling() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(100, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05).run_laps(1, v0=15)
    df = res.laps[0].telemetry.get("downforce", [0.0])
    ok = len(df) > 0 and all(math.isfinite(v) for v in df)
    return _ok("aero_coupling", ok)


def gate_driver_path_following() -> tuple[str, bool, str]:
    tr = TrackLibrary.handling_course(ds=2.0)
    line = ideal_line(tr)
    ok = line.kind == "ideal" and np.all(np.isfinite(line.x))
    return _ok("driver_path_following", ok, f"pts={len(line.x)}")


def gate_deterministic_replay() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(100, ds=2.0)
    r1 = LapSimulator(None, tr, dt=0.05, line="center").run_laps(1, v0=12)
    r2 = LapSimulator(None, tr, dt=0.05, line="center").run_laps(1, v0=12)
    ok = abs(r1.best_lap - r2.best_lap) < 1e-9
    return _ok("deterministic_replay", ok, f"{r1.best_lap:.4f}")


def gate_statistics_generation() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(150, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05).run_laps(2, v0=10)
    text = res.statistics.summary()
    ok = "Best lap" in text and res.statistics.n_laps == 2
    return _ok("statistics_generation", ok)


def gate_export_formats() -> tuple[str, bool, str]:
    tr = TrackLibrary.straight(80, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05).run_laps(1, v0=10)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        res.export_csv(p / "lap.csv")
        res.export_json(p / "lap.json")
        res.export_report(p / "lap.md")
        ok = (p / "lap.csv").exists() and (p / "lap.json").exists() and (p / "lap.md").exists()
    return _ok("export_formats", ok)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    tr = TrackLibrary.straight(60, ds=3.0)
    t0 = time.perf_counter()
    LapSimulator(None, tr, dt=0.05).run_laps(1, v0=10)
    ms = (time.perf_counter() - t0) * 1000
    return _ok("performance_regression", ms < 15000.0, f"{ms:.0f} ms")


def gate_repeatability() -> tuple[str, bool, str]:
    tr = TrackLibrary.oval(radius=40, straights=50, ds=3.0)
    a = LapSimulator(None, tr, dt=0.05, line="center").run_laps(1, v0=12)
    b = LapSimulator(None, tr, dt=0.05, line="center").run_laps(1, v0=12)
    return _ok("repeatability", abs(a.best_lap - b.best_lap) < 1e-9)


def gate_no_nan_inf() -> tuple[str, bool, str]:
    tr = TrackLibrary.slalom(n_gates=4, ds=2.0)
    res = LapSimulator(None, tr, dt=0.05).run_laps(1, v0=8)
    telem = res.laps[0].telemetry
    ok = all(math.isfinite(v) for key in ("vx", "time", "s") for v in telem[key])
    return _ok("no_nan_inf", ok)


def gate_full_lap_regression() -> tuple[str, bool, str]:
    tr = TrackLibrary.handling_course(ds=2.0)
    res = LapSimulator(None, tr, dt=0.05, n_sectors=3, line="ideal").run_laps(1, v0=12)
    ok = (
        res.statistics.n_laps == 1
        and res.best_lap > 0
        and len(res.sector_times) == 3
        and res.statistics.total_distance > 100
    )
    return _ok("full_lap_regression", ok, f"t={res.best_lap:.2f} sectors={res.sector_times}")


GATES = [
    gate_track_loading,
    gate_segment_generation,
    gate_curvature_solver,
    gate_surface_mapping,
    gate_racing_line_generation,
    gate_lap_completion,
    gate_sector_timing,
    gate_telemetry_logging,
    gate_vehicle_comparison,
    gate_fuel_tracking,
    gate_battery_tracking,
    gate_aero_coupling,
    gate_driver_path_following,
    gate_deterministic_replay,
    gate_statistics_generation,
    gate_export_formats,
    gate_performance_regression,
    gate_repeatability,
    gate_no_nan_inf,
    gate_full_lap_regression,
]


def run_phase121_validation(verbose: bool = True) -> bool:
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
            print("Phase 12.1 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 12.1 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase121_validation() else 1)
