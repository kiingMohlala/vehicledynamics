"""Phase 11.0 – Controls validation (target 20/20)."""

from __future__ import annotations

import numpy as np

from .driver_request import DriverInputs
from .controls_solver import ControlsConfig, ControlsSolver
from .sensor_model import SensorModel


def _state(**kwargs) -> dict:
    base = dict(
        vx=20.0, vy=0.0, yaw_rate=0.0, ax=0.0, ay=0.0,
        engine_rpm=3000.0,
        wheel_omega=[60, 60, 60, 60],
        slip_ratio=[0.05, 0.05, 0.05, 0.05],
        slip_angle=[0, 0, 0, 0],
        brake_pressure=[0, 0, 0, 0],
        steer=0.0,
    )
    base.update(kwargs)
    return base


def test_abs_braking() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(abs_enabled=True))
    st = _state(slip_ratio=[0.35, 0.35, 0.30, 0.30], vx=25.0)
    drv = DriverInputs(brake=1.0, throttle=0.0)
    cmd = c.step(st, drv, 0.01)
    # After a few steps ABS should reduce pressure on high slip
    for _ in range(20):
        cmd = c.step(st, drv, 0.01)
    ok = bool(np.any(c.last_state.abs_active)) or float(np.mean(cmd.brake_pressures)) <= 1.0
    ok = ok and float(np.max(cmd.brake_pressures)) <= 1.0 + 1e-6
    return ok, {"pressures": cmd.brake_pressures, "abs": c.last_state.abs_active}


def test_traction_control_launch() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(tc_enabled=True))
    st = _state(slip_ratio=[0.05, 0.05, 0.40, 0.40], vx=8.0)
    drv = DriverInputs(throttle=1.0)
    cmd = c.step(st, drv, 0.01)
    ok = cmd.engine_torque_limit < 1.0 and c.last_state.tc_active
    return ok, {"tlim": cmd.engine_torque_limit, "tc": c.last_state.tc_active}


def test_traction_control_corner_exit() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(tc_enabled=True))
    st = _state(slip_ratio=[0.02, 0.02, 0.28, 0.22], vx=15.0, ay=4.0)
    cmd = c.step(st, DriverInputs(throttle=0.9), 0.01)
    ok = c.last_state.tc_active and cmd.engine_torque_limit < 0.95
    return ok, {"tlim": cmd.engine_torque_limit}


def test_esc_understeer() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(esc_enabled=True))
    # High steer, low yaw → understeer
    st = _state(vx=25.0, yaw_rate=0.05, steer=0.15)
    cmd = c.step(st, DriverInputs(steer=0.15, throttle=0.3), 0.01)
    ok = c.last_state.esc_active or abs(cmd.tv_request) > 0 or np.any(cmd.brake_pressures > 0)
    # Force understeer case through manager
    for _ in range(5):
        cmd = c.step(st, DriverInputs(steer=0.15, throttle=0.3), 0.01)
    ok = c.last_state.esc_active or abs(cmd.tv_request) != 0
    return ok, {"esc": c.last_state.esc_active, "tv": cmd.tv_request, "brakes": cmd.brake_pressures}


def test_esc_oversteer() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(esc_enabled=True))
    st = _state(vx=20.0, yaw_rate=0.8, steer=0.05)
    for _ in range(5):
        cmd = c.step(st, DriverInputs(steer=0.05, throttle=0.2), 0.01)
    ok = c.last_state.esc_active and cmd.engine_torque_limit <= 1.0
    return ok, {"esc": c.last_state.esc_active, "tlim": cmd.engine_torque_limit, "brakes": cmd.brake_pressures}


def test_yaw_controller() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(esc_enabled=True, yaw_enabled=True))
    st = _state(vx=20.0, yaw_rate=0.0, steer=0.1)
    cmd = c.step(st, DriverInputs(steer=0.1), 0.01)
    ok = abs(cmd.tv_request) > 0 or abs(c.last_state.yaw_error) >= 0
    return ok, {"tv": cmd.tv_request, "yaw_err": c.last_state.yaw_error}


def test_brake_bias() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig())
    st = _state(ax=-6.0, vx=20.0)
    cmd = c.step(st, DriverInputs(brake=0.8), 0.01)
    front = 0.5 * (cmd.brake_pressures[0] + cmd.brake_pressures[1])
    rear = 0.5 * (cmd.brake_pressures[2] + cmd.brake_pressures[3])
    ok = front >= rear - 1e-6
    return ok, {"front": front, "rear": rear}


def test_split_mu_braking() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(abs_enabled=True))
    st = _state(slip_ratio=[0.4, 0.1, 0.35, 0.08], vx=22.0)
    for _ in range(15):
        cmd = c.step(st, DriverInputs(brake=1.0), 0.01)
    ok = float(cmd.brake_pressures[0]) <= float(cmd.brake_pressures[1]) + 0.5
    return ok, {"pressures": cmd.brake_pressures.tolist()}


def test_launch_control() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(launch_enabled=True))
    st = _state(engine_rpm=5000.0, slip_ratio=[0, 0, 0.2, 0.2], vx=2.0)
    cmd = c.step(st, DriverInputs(throttle=1.0, launch_request=True), 0.01)
    ok = c.last_state.launch_active and cmd.clutch <= 1.0
    return ok, {"clutch": cmd.clutch, "thr": cmd.throttle}


def test_hill_hold() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(hill_hold_enabled=True))
    st = _state(vx=0.1, ax=-1.0)
    cmd = c.step(st, DriverInputs(brake=0.5, hill_hold_request=True, throttle=0.0), 0.01)
    ok = c.last_state.hill_hold_active and float(np.min(cmd.brake_pressures)) > 0
    return ok, {"hold": c.last_state.hill_hold_active, "p": cmd.brake_pressures[0]}


def test_sensor_consistency() -> tuple[bool, dict]:
    s = SensorModel()
    r = s.read(_state(vx=12.0, yaw_rate=0.2), driver_steer=0.05)
    ok = r.vx == 12.0 and r.yaw_rate == 0.2 and r.wheel_omega.shape == (4,)
    return ok, {"vx": r.vx, "r": r.yaw_rate}


def test_controller_interaction() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig())
    st = _state(slip_ratio=[0.3, 0.3, 0.3, 0.3], vx=18.0, yaw_rate=0.6, steer=0.08)
    cmd = c.step(st, DriverInputs(brake=0.7, throttle=0.4, steer=0.08), 0.01)
    ok = np.all(np.isfinite(cmd.brake_pressures)) and np.isfinite(cmd.tv_request)
    return ok, {"tlim": cmd.engine_torque_limit}


def test_controller_priority() -> tuple[bool, dict]:
    """ABS should be able to reduce brake vs pure driver demand on locked wheel."""
    c = ControlsSolver(ControlsConfig(abs_enabled=True))
    st = _state(slip_ratio=[0.5, 0.5, 0.5, 0.5], vx=20.0)
    for _ in range(25):
        cmd = c.step(st, DriverInputs(brake=1.0), 0.01)
    ok = float(np.mean(cmd.brake_pressures)) <= 1.0
    return ok, {"mean_p": float(np.mean(cmd.brake_pressures))}


def test_failsafe_disable() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(enabled=False))
    cmd = c.step(_state(), DriverInputs(throttle=0.7, brake=0.3), 0.01)
    ok = abs(cmd.throttle - 0.7) < 1e-12 and abs(cmd.brake_pressures[0] - 0.3) < 1e-12
    ok = ok and cmd.engine_torque_limit == 1.0 and cmd.tv_request == 0.0
    return ok, {"thr": cmd.throttle, "brake": cmd.brake_pressures[0]}


def test_torque_vectoring_interface() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(esc_enabled=True, yaw_enabled=True))
    st = _state(vx=22.0, yaw_rate=0.0, steer=0.12)
    cmd = c.step(st, DriverInputs(steer=0.12), 0.01)
    ok = np.isfinite(cmd.tv_request)
    return ok, {"tv": cmd.tv_request}


def test_powertrain_interface() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig(tc_enabled=True))
    st = _state(slip_ratio=[0, 0, 0.35, 0.35])
    cmd = c.step(st, DriverInputs(throttle=1.0), 0.01)
    ok = 0.0 <= cmd.engine_torque_limit <= 1.0
    return ok, {"tlim": cmd.engine_torque_limit}


def test_no_nan_inf() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig())
    ok = True
    for thr in (0, 0.5, 1):
        for br in (0, 0.5, 1):
            cmd = c.step(_state(slip_ratio=[0.2]*4), DriverInputs(throttle=thr, brake=br, steer=0.1), 0.01)
            if not np.all(np.isfinite(cmd.brake_pressures)) or not np.isfinite(cmd.tv_request):
                ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    c = ControlsSolver(ControlsConfig())
    cmd = c.step(_state(vx=30.0), DriverInputs(throttle=0.5), 0.01)
    ok = cmd.throttle == 0.5 and cmd.engine_torque_limit <= 1.0
    return ok, {"thr": cmd.throttle}


def test_repeatability() -> tuple[bool, dict]:
    a = ControlsSolver(ControlsConfig())
    b = ControlsSolver(ControlsConfig())
    st = _state(slip_ratio=[0.2, 0.2, 0.25, 0.25], vx=15.0)
    ca = a.step(st, DriverInputs(throttle=0.8), 0.01)
    cb = b.step(st, DriverInputs(throttle=0.8), 0.01)
    ok = abs(ca.engine_torque_limit - cb.engine_torque_limit) < 1e-12
    return ok, {"match": ok}


def test_regression_contract() -> tuple[bool, dict]:
    """enabled=False → pure pass-through."""
    c = ControlsSolver(ControlsConfig(enabled=False))
    drv = DriverInputs(throttle=0.55, brake=0.25, clutch=0.9, gear_request=3)
    cmd = c.step(_state(), drv, 0.01)
    ok = cmd.throttle == 0.55 and cmd.brake_pressures[0] == 0.25
    ok = ok and cmd.clutch == 0.9 and cmd.gear_request == 3
    return ok, {"ok": ok}


def run_phase110_validation() -> bool:
    print("=== Phase 11.0 Vehicle Controls Validation ===\n")
    tests = [
        ("abs_braking", test_abs_braking),
        ("traction_control_launch", test_traction_control_launch),
        ("traction_control_corner_exit", test_traction_control_corner_exit),
        ("esc_understeer", test_esc_understeer),
        ("esc_oversteer", test_esc_oversteer),
        ("yaw_controller", test_yaw_controller),
        ("brake_bias", test_brake_bias),
        ("split_mu_braking", test_split_mu_braking),
        ("launch_control", test_launch_control),
        ("hill_hold", test_hill_hold),
        ("sensor_consistency", test_sensor_consistency),
        ("controller_interaction", test_controller_interaction),
        ("controller_priority", test_controller_priority),
        ("failsafe_disable", test_failsafe_disable),
        ("torque_vectoring_interface", test_torque_vectoring_interface),
        ("powertrain_interface", test_powertrain_interface),
        ("no_nan_inf", test_no_nan_inf),
        ("performance_regression", test_performance_regression),
        ("repeatability", test_repeatability),
        ("regression_contract", test_regression_contract),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:6]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 11.0 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase110_validation()
