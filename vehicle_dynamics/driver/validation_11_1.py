"""Phase 11.1 – Driver model validation (target 22/22)."""

from __future__ import annotations

import tempfile
from pathlib import Path
import numpy as np

from .reference_paths import make_straight, make_circle, make_slalom, make_figure_eight
from .steering_controller import PurePursuit, StanleyController, SteeringPID
from .speed_controller import SpeedController
from .path_follower import PathFollower
from .maneuver_library import ManeuverLibrary
from .driver_model import DriverConfig, DriverModel
from .driver_solver import DriverSolver
from .telemetry import TelemetryLogger, TelemetrySample
from vehicle_dynamics.controls.driver_request import DriverInputs


def _sim_path(follower: PathFollower, n: int = 200, dt: float = 0.05, v: float = 15.0):
    """Simple kinematic bicycle integration for path tracking tests."""
    x, y, psi = 0.0, 0.5, 0.0  # slight offset
    L = 2.7
    ctes = []
    for _ in range(n):
        st = follower.step(x, y, psi, v, dt)
        ctes.append(st.cross_track)
        # bicycle
        x += v * np.cos(psi) * dt
        y += v * np.sin(psi) * dt
        psi += (v / L) * np.tan(st.steer) * dt
    return np.array(ctes), st


def test_straight_line_tracking() -> tuple[bool, dict]:
    path = make_straight(150.0, v_ref=20.0)
    f = PathFollower(path=path, mode="pure_pursuit", lookahead=10.0)
    ctes, _ = _sim_path(f, n=150, v=18.0)
    # Should reduce CTE
    ok = abs(ctes[-1]) < abs(ctes[0]) or abs(ctes[-1]) < 2.0
    return ok, {"cte0": ctes[0], "cte_end": ctes[-1]}


def test_constant_speed_tracking() -> tuple[bool, dict]:
    sp = SpeedController()
    v = 10.0
    for _ in range(80):
        thr, br = sp.step(v, 20.0, 0.05)
        v += (thr * 3.0 - br * 5.0) * 0.05  # crude accel model
    ok = abs(v - 20.0) < 3.0
    return ok, {"v": v}


def test_pid_speed_control() -> tuple[bool, dict]:
    sp = SpeedController(kp=0.2, ki=0.05)
    thr, br = sp.step(10.0, 15.0, 0.01)
    ok = thr > 0 and br == 0
    thr2, br2 = sp.step(20.0, 15.0, 0.01)
    ok = ok and br2 > 0
    return ok, {"thr_up": thr, "br_down": br2}


def test_pure_pursuit_tracking() -> tuple[bool, dict]:
    path = make_circle(radius=40.0, v_ref=12.0)
    f = PathFollower(path=path, mode="pure_pursuit", lookahead=8.0)
    # Start on path
    p0 = path.sample(0.0)
    x, y, psi, v = p0.x, p0.y, p0.psi, 12.0
    L = 2.7
    errs = []
    for _ in range(100):
        st = f.step(x, y, psi, v, 0.05)
        errs.append(abs(st.cross_track))
        x += v * np.cos(psi) * 0.05
        y += v * np.sin(psi) * 0.05
        psi += (v / L) * np.tan(st.steer) * 0.05
    ok = float(np.mean(errs[-20:])) < 5.0
    return ok, {"mean_cte_tail": float(np.mean(errs[-20:]))}


def test_stanley_tracking() -> tuple[bool, dict]:
    path = make_straight(100.0, v_ref=12.0)
    f = PathFollower(path=path, mode="stanley", lookahead=6.0)
    f.stanley.k = 0.8
    # Custom sim with small offset
    x, y, psi, v, L, dt = 0.0, 0.3, 0.0, 12.0, 2.7, 0.05
    ctes = []
    for _ in range(150):
        st = f.step(x, y, psi, v, dt)
        ctes.append(st.cross_track)
        x += v * __import__("numpy").cos(psi) * dt
        y += v * __import__("numpy").sin(psi) * dt
        psi += (v / L) * __import__("numpy").tan(st.steer) * dt
    ok = abs(ctes[-1]) < 1.5 or abs(ctes[-1]) < abs(ctes[0])
    return ok, {"cte0": ctes[0], "cte_end": ctes[-1], "mode": st.mode}


def test_circle_path() -> tuple[bool, dict]:
    p = make_circle(30.0)
    ok = p.length > 100.0 and len(p.points) > 10
    return ok, {"length": p.length}


def test_slalom_path() -> tuple[bool, dict]:
    p = make_slalom()
    ys = [pt.y for pt in p.points]
    ok = max(ys) > 1.0 and min(ys) < -1.0
    return ok, {"ymax": max(ys), "ymin": min(ys)}


def test_figure_eight() -> tuple[bool, dict]:
    p = make_figure_eight()
    ok = p.length > 50.0
    return ok, {"length": p.length}


def test_double_lane_change() -> tuple[bool, dict]:
    m = ManeuverLibrary.double_lane_change()
    ok = m.path is not None and m.path.length > 100
    return ok, {"length": m.path.length if m.path else 0}


def test_step_steer() -> tuple[bool, dict]:
    m = ManeuverLibrary.step_steer(0.1)
    thr, br, st = m.profile.at(2.0)
    ok = abs(st - 0.1) < 1e-6
    return ok, {"steer": st}


def test_fishhook() -> tuple[bool, dict]:
    m = ManeuverLibrary.fishhook()
    _, _, st = m.profile.at(3.0)
    ok = st < 0  # second phase negative
    return ok, {"steer": st}


def test_emergency_braking() -> tuple[bool, dict]:
    m = ManeuverLibrary.emergency_braking()
    _, br, _ = m.profile.at(1.0)
    ok = br > 0.9
    return ok, {"brake": br}


def test_launch_test() -> tuple[bool, dict]:
    m = ManeuverLibrary.launch_test()
    thr, _, _ = m.profile.at(1.0)
    ok = thr > 0.9
    return ok, {"throttle": thr}


def test_controller_integration() -> tuple[bool, dict]:
    sol = DriverSolver(DriverConfig(mode="pure_pursuit"))
    sol.set_path(make_straight(50.0, v_ref=15.0))
    pose = {"x": 0.0, "y": 0.2, "psi": 0.0, "vx": 12.0}
    inp = sol.step(pose, 0.01)
    ok = isinstance(inp, DriverInputs) and np.isfinite(inp.steer)
    return ok, {"steer": inp.steer, "thr": inp.throttle}


def test_controls_interaction() -> tuple[bool, dict]:
    """Driver output is valid DriverInputs for Phase 11.0."""
    from vehicle_dynamics.controls import ControlsSolver, ControlsConfig
    sol = DriverSolver(DriverConfig(mode="stanley"))
    sol.set_path(make_straight(80.0))
    drv = sol.step({"x": 1.0, "y": 0.0, "psi": 0.0, "vx": 15.0}, 0.01)
    ctrl = ControlsSolver(ControlsConfig(enabled=True))
    cmd = ctrl.step({"vx": 15.0, "slip_ratio": [0.05]*4, "steer": drv.steer}, drv, 0.01)
    ok = np.isfinite(cmd.throttle) and np.all(np.isfinite(cmd.brake_pressures))
    return ok, {"thr": cmd.throttle}


def test_cross_track_error() -> tuple[bool, dict]:
    path = make_straight()
    _, cte = path.nearest(10.0, 2.0)
    ok = abs(cte - 2.0) < 0.5
    return ok, {"cte": cte}


def test_heading_error() -> tuple[bool, dict]:
    f = PathFollower(path=make_straight(), mode="pid")
    st = f.step(0.0, 0.0, 0.3, 10.0, 0.01)  # psi offset
    ok = abs(st.heading_error) > 0.1
    return ok, {"herr": st.heading_error}


def test_telemetry_logging() -> tuple[bool, dict]:
    log = TelemetryLogger()
    log.log(TelemetrySample(time=0.0, cross_track=1.0))
    log.log(TelemetrySample(time=0.1, cross_track=-1.0))
    ok = len(log.samples) == 2 and abs(log.rms_cross_track - 1.0) < 1e-9
    return ok, {"rms": log.rms_cross_track}


def test_csv_export() -> tuple[bool, dict]:
    log = TelemetryLogger()
    log.log(TelemetrySample(time=0.0, vx=10.0, throttle=0.5))
    with tempfile.TemporaryDirectory() as td:
        p = log.export_csv(Path(td) / "telem.csv")
        ok = p.exists() and p.stat().st_size > 20
        size = p.stat().st_size if p.exists() else 0
    return ok, {"bytes": size}


def test_repeatability() -> tuple[bool, dict]:
    def run():
        sol = DriverSolver(DriverConfig(mode="pure_pursuit", lookahead=8.0))
        sol.set_path(make_straight(40.0, v_ref=15.0))
        return sol.step({"x": 0.0, "y": 0.5, "psi": 0.0, "vx": 15.0}, 0.01).steer
    ok = abs(run() - run()) < 1e-12
    return ok, {"match": ok}


def test_no_nan_inf() -> tuple[bool, dict]:
    sol = DriverSolver(DriverConfig())
    ok = True
    for mode in ("pure_pursuit", "stanley", "pid"):
        sol.model.cfg.mode = mode
        sol.set_path(make_circle(25.0))
        for i in range(20):
            inp = sol.step({"x": float(i), "y": 0.1, "psi": 0.0, "vx": 12.0}, 0.05)
            if not all(np.isfinite([inp.throttle, inp.brake, inp.steer])):
                ok = False
    return ok, {"ok": ok}


def test_regression_contract() -> tuple[bool, dict]:
    sol = DriverSolver(DriverConfig(enabled=False))
    ext = DriverInputs(throttle=0.4, brake=0.1, steer=0.05)
    out = sol.step({"x": 0, "y": 0, "psi": 0, "vx": 10}, 0.01, external=ext)
    ok = out.throttle == 0.4 and out.brake == 0.1 and out.steer == 0.05
    return ok, {"ok": ok}


def run_phase111_validation() -> bool:
    print("=== Phase 11.1 Driver Model Validation ===\n")
    tests = [
        ("straight_line_tracking", test_straight_line_tracking),
        ("constant_speed_tracking", test_constant_speed_tracking),
        ("pid_speed_control", test_pid_speed_control),
        ("pure_pursuit_tracking", test_pure_pursuit_tracking),
        ("stanley_tracking", test_stanley_tracking),
        ("circle_path", test_circle_path),
        ("slalom_path", test_slalom_path),
        ("figure_eight", test_figure_eight),
        ("double_lane_change", test_double_lane_change),
        ("step_steer", test_step_steer),
        ("fishhook", test_fishhook),
        ("emergency_braking", test_emergency_braking),
        ("launch_test", test_launch_test),
        ("controller_integration", test_controller_integration),
        ("controls_interaction", test_controls_interaction),
        ("cross_track_error", test_cross_track_error),
        ("heading_error", test_heading_error),
        ("telemetry_logging", test_telemetry_logging),
        ("csv_export", test_csv_export),
        ("repeatability", test_repeatability),
        ("no_nan_inf", test_no_nan_inf),
        ("regression_contract", test_regression_contract),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:5]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 11.1 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase111_validation()
