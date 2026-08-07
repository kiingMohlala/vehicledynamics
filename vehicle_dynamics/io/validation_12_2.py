"""Phase 12.2 – Open Simulation Interface & Standards validation (20 gates)."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np

from .opendrive import load_opendrive, write_minimal_opendrive
from .openscenario import load_openscenario, write_minimal_openscenario
from .csv_import import load_csv_columns, load_xy_path
from .telemetry_import import load_telemetry_csv, compare_traces
from .ros2_bridge import ROS2Bridge
from .fmu_export import export_fmu, read_model_description
from .can_bus import CANBus
from .sensor_export import SensorExporter, SensorConfig
from .coordinate_frames import iso_to_sae, sae_to_iso, enu_to_ned, transform
from .unit_conversion import convert, kmh_to_ms, ms_to_kmh, deg_to_rad
from .project_exchange import export_project, load_project


def _ok(name: str, passed: bool, detail: str = "") -> tuple[str, bool, str]:
    return name, passed, detail


def gate_opendrive_import() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = write_minimal_opendrive(Path(td) / "test.xodr", length=250.0)
        tr = load_opendrive(p, ds=2.0)
    return _ok("opendrive_import", tr.length > 100, f"L={tr.length:.1f}")


def gate_openscenario_import() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = write_minimal_openscenario(Path(td) / "dlc.json", name="double_lane_change")
        sc = load_openscenario(p)
    return _ok("openscenario_import", sc.name == "double_lane_change" and len(sc.events) >= 1, sc.name)


def gate_csv_import() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "path.csv"
        p.write_text("x,y\n0,0\n10,0\n20,5\n")
        x, y = load_xy_path(p)
    return _ok("csv_import", len(x) == 3 and abs(x[-1] - 20) < 1e-9)


def gate_telemetry_alignment() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "log.csv"
        p.write_text("time,vx\n0,0\n1,10\n2,20\n")
        log = load_telemetry_csv(p)
        aligned = log.align_to(np.array([0.5, 1.5]))
    ok = abs(aligned["vx"][0] - 5.0) < 1e-9 and abs(aligned["vx"][1] - 15.0) < 1e-9
    return _ok("telemetry_alignment", ok)


def gate_ros_publish() -> tuple[str, bool, str]:
    ros = ROS2Bridge()
    ros.publish_vehicle_state({"vx": 12.0, "engine_rpm": 3000}, t=1.0)
    msg = ros.latest(ROS2Bridge.TOPIC_VEHICLE_STATE)
    return _ok("ros_publish", msg is not None and msg.data["vx"] == 12.0)


def gate_ros_subscribe() -> tuple[str, bool, str]:
    ros = ROS2Bridge()
    received = []
    ros.subscribe(ROS2Bridge.TOPIC_CMD_THROTTLE, lambda m: received.append(m.data))
    ros.inject_command(ROS2Bridge.TOPIC_CMD_THROTTLE, {"throttle": 0.4}, t=0.1)
    return _ok("ros_subscribe", len(received) == 1 and received[0]["throttle"] == 0.4)


def gate_fmu_export() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = export_fmu(Path(td) / "vehicle.fmu", model_name="TestVehicle")
        info = read_model_description(p)
    return _ok("fmu_export", info["has_cosim"] and info["has_throttle"] and info["has_vx"], info["fmiVersion"])


def gate_can_messages() -> tuple[str, bool, str]:
    can = CANBus()
    msgs = can.publish({"engine_rpm": 4500, "throttle": 0.5, "steer": 0.1, "brake": 0.0}, t=0.0)
    ok = len(msgs) >= 3
    dec = can.decode(msgs[0])
    ok = ok and "engine_rpm" in dec and dec["engine_rpm"] == 4500
    return _ok("can_messages", ok, f"n={len(msgs)}")


def gate_imu_generation() -> tuple[str, bool, str]:
    exp = SensorExporter(SensorConfig(imu_noise_std=0.0, seed=1))
    imu = exp.imu(1.0, -0.5, 9.81, gz=0.2)
    ok = abs(imu["ax"] - 1.0) < 1e-12 and abs(imu["gz"] - 0.2) < 1e-12
    return _ok("imu_generation", ok)


def gate_gps_generation() -> tuple[str, bool, str]:
    exp = SensorExporter(SensorConfig(gps_noise_std=0.0, seed=1))
    g = exp.gps(100.0, 50.0, 1.0)
    return _ok("gps_generation", abs(g["x"] - 100) < 1e-12 and abs(g["y"] - 50) < 1e-12)


def gate_coordinate_transforms() -> tuple[str, bool, str]:
    v = np.array([1.0, 2.0, 3.0])
    sae = iso_to_sae(v)
    back = sae_to_iso(sae)
    ned = enu_to_ned(np.array([1.0, 2.0, 3.0]))
    ok = np.allclose(back, v) and np.allclose(ned, [2.0, 1.0, -3.0])
    t = transform(v, "iso", "sae")
    ok = ok and np.allclose(t, sae)
    return _ok("coordinate_transforms", ok)


def gate_sensor_noise() -> tuple[str, bool, str]:
    exp = SensorExporter(SensorConfig(imu_noise_std=0.1, seed=42))
    samples = [exp.imu(0.0, 0.0, 0.0)["ax"] for _ in range(50)]
    std = float(np.std(samples))
    return _ok("sensor_noise", 0.05 < std < 0.2, f"std={std:.3f}")


def gate_project_exchange() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td) / "proj"
        export_project(
            root,
            vehicle={"name": "sedan"},
            track={"name": "oval", "length": 1000},
            results={"best_lap": 90.5},
            reports={"summary.md": "# OK"},
            meta={"phase": "12.2"},
        )
        data = load_project(root)
    ok = data["vehicle"]["name"] == "sedan" and data["manifest"]["format"].startswith("vehicle_dynamics")
    return _ok("project_exchange", ok)


def gate_unit_conversion() -> tuple[str, bool, str]:
    ok = abs(ms_to_kmh(10) - 36.0) < 1e-9
    ok = ok and abs(kmh_to_ms(36) - 10.0) < 1e-9
    ok = ok and abs(convert(180, "deg", "rad") - math.pi) < 1e-9
    return _ok("unit_conversion", ok)


def gate_large_track_import() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        # long straight road
        p = write_minimal_opendrive(Path(td) / "long.xodr", length=5000.0)
        tr = load_opendrive(p, ds=5.0)
    return _ok("large_track_import", tr.length > 4000, f"L={tr.length:.0f}")


def gate_replay_consistency() -> tuple[str, bool, str]:
    can = CANBus()
    state = {"engine_rpm": 3000, "throttle": 0.3, "steer": -0.05, "brake": 0.1, "battery_soc": 0.8}
    m1 = can.publish(state, t=0.0)
    decoded = {}
    for m in m1:
        decoded.update(can.decode(m))
    ok = decoded["engine_rpm"] == 3000 and abs(decoded["throttle"] - 0.3) < 1e-9
    return _ok("replay_consistency", ok)


def gate_performance_regression() -> tuple[str, bool, str]:
    import time
    with tempfile.TemporaryDirectory() as td:
        p = write_minimal_opendrive(Path(td) / "p.xodr", length=500.0)
        t0 = time.perf_counter()
        for _ in range(20):
            load_opendrive(p, ds=5.0)
        ms = (time.perf_counter() - t0) / 20 * 1000
    return _ok("performance_regression", ms < 200.0, f"{ms:.2f} ms")


def gate_repeatability() -> tuple[str, bool, str]:
    with tempfile.TemporaryDirectory() as td:
        p = write_minimal_opendrive(Path(td) / "r.xodr", length=300.0)
        a = load_opendrive(p, ds=2.0).length
        b = load_opendrive(p, ds=2.0).length
    return _ok("repeatability", abs(a - b) < 1e-12)


def gate_no_nan_inf() -> tuple[str, bool, str]:
    exp = SensorExporter(SensorConfig(seed=0))
    out = exp.export({"ax": 1.0, "ay": 0.0, "x": 0.0, "y": 0.0, "vx": 10.0})
    vals = list(out["imu"].values()) + list(out["gps"].values()) + out["wheel_speeds"]
    return _ok("no_nan_inf", all(math.isfinite(v) for v in vals))


def gate_regression_contract() -> tuple[str, bool, str]:
    # I/O layer must not alter physics modules; smoke-import simulation still works
    try:
        from vehicle_dynamics.simulation import Simulation, SimulationConfig, ScenarioLibrary
        sim = Simulation(SimulationConfig(dt=0.02))
        sim.load_scenario(ScenarioLibrary.straight_acceleration(0.5))
        r = sim.run(0.5)
        ok = len(r.telemetry.samples) > 0
        return _ok("regression_contract", ok)
    except Exception as e:
        return _ok("regression_contract", False, str(e))


GATES = [
    gate_opendrive_import,
    gate_openscenario_import,
    gate_csv_import,
    gate_telemetry_alignment,
    gate_ros_publish,
    gate_ros_subscribe,
    gate_fmu_export,
    gate_can_messages,
    gate_imu_generation,
    gate_gps_generation,
    gate_coordinate_transforms,
    gate_sensor_noise,
    gate_project_exchange,
    gate_unit_conversion,
    gate_large_track_import,
    gate_replay_consistency,
    gate_performance_regression,
    gate_repeatability,
    gate_no_nan_inf,
    gate_regression_contract,
]


def run_phase122_validation(verbose: bool = True) -> bool:
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
            print("Phase 12.2 Status: IMPLEMENTATION VALIDATED ✓")
        else:
            print(f"FAILED: {n_pass}/{n}")
            print("Phase 12.2 Status: NOT VALIDATED")
    return n_pass == n


if __name__ == "__main__":
    raise SystemExit(0 if run_phase122_validation() else 1)
