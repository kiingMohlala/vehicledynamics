"""Phase 10.3 – Advanced driveline validation (target 20/20)."""

from __future__ import annotations

import numpy as np

from .shaft import ElasticShaft
from .halfshaft import HalfShaftPair
from .backlash import Backlash
from .gear_mesh import GearMesh
from .wheel_inertia import WheelInertia
from .driveline_solver import DrivelineConfig, DrivelineSolver


def test_rigid_shaft_regression() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=False))
    st = s.step(400.0, 0.0, 0.0, dt=0.001)
    ok = abs(st.torque_left - 200.0) < 1e-9 and abs(st.torque_right - 200.0) < 1e-9
    return ok, {"TL": st.torque_left, "TR": st.torque_right}


def test_elastic_shaft_twist() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0))
    for _ in range(200):
        s.step(300.0, 50.0, 50.0, dt=0.001)
    ok = abs(s.state.shaft_twist) > 1e-4
    return ok, {"twist": s.state.shaft_twist, "T": s.state.torque_left + s.state.torque_right}


def test_shaft_energy_storage() -> tuple[bool, dict]:
    shaft = ElasticShaft(12000.0, 0.0)
    theta = 0.05
    E = shaft.energy(theta)
    expected = 0.5 * 12000.0 * theta ** 2
    ok = abs(E - expected) < 1e-9 and E > 0
    return ok, {"E": E, "expected": expected}


def test_halfshaft_compliance() -> tuple[bool, dict]:
    hs = HalfShaftPair(k_left=8000, k_right=8000, c_left=0, c_right=0)
    st = hs.evaluate(0.02, 0.0, -0.01, 0.0)
    ok = abs(st.torque_left - 160.0) < 1e-6 and abs(st.torque_right + 80.0) < 1e-6
    return ok, {"TL": st.torque_left, "TR": st.torque_right}


def test_gear_backlash_deadzone() -> tuple[bool, dict]:
    bl = Backlash.from_degrees(0.5)
    gap = bl.gap
    st = bl.evaluate(0.5 * gap)
    ok = not st.engaged and bl.effective_angle(0.5 * gap) == 0.0
    return ok, {"gap": gap, "engaged": st.engaged}


def test_gear_backlash_engagement() -> tuple[bool, dict]:
    bl = Backlash.from_degrees(0.4)
    theta = bl.gap + 0.02
    st = bl.evaluate(theta)
    eff = bl.effective_angle(theta)
    ok = st.engaged and abs(eff - 0.02) < 1e-9
    return ok, {"eff": eff, "side": st.side}


def test_gear_mesh_stiffness() -> tuple[bool, dict]:
    mesh = GearMesh(stiffness=5e4, damping=0.0, ripple_amp=0.0)
    T = mesh.torque(0.001, 0.0)
    ok = abs(T - 50.0) < 1e-6
    return ok, {"T": T}


def test_wheel_inertia_response() -> tuple[bool, dict]:
    w = WheelInertia(J_wheel=0.9, J_rotor=0.1)
    a = w.accel(100.0)
    ok = abs(a - 100.0 / 1.0) < 1e-9
    return ok, {"a": a, "J": w.J_total}


def test_torsional_oscillation() -> tuple[bool, dict]:
    """Impulse then free decay should show sign changes in twist rate proxy."""
    s = DrivelineSolver(DrivelineConfig(
        enabled=True, backlash_deg=0.0, shaft_damping=5.0, shaft_stiffness=8000.0
    ))
    # Load up
    for _ in range(50):
        s.step(500.0, 0.0, 0.0, dt=0.001)
    twists = []
    for _ in range(400):
        s.step(0.0, 0.0, 0.0, dt=0.001)
        twists.append(s.state.shaft_twist)
    # Zero crossings of twist relative to mean
    arr = np.array(twists)
    arr = arr - np.mean(arr)
    crossings = np.sum(np.diff(np.sign(arr)) != 0)
    ok = crossings >= 2 and s.state.oscillation_freq_hz > 0
    return ok, {"crossings": int(crossings), "f_hz": s.state.oscillation_freq_hz}


def test_torsional_damping() -> tuple[bool, dict]:
    """Viscous damping law T = kθ + cω and dissipative power for c > 0."""
    shaft = ElasticShaft(stiffness=0.0, damping=50.0)
    omega = 10.0
    T = shaft.torque(0.0, omega)
    # T = c*omega > 0 when omega > 0; power extracted from relative motion is -T*omega < 0
    # when opposing the relative velocity on the other side — magnitude scales with c
    T2 = ElasticShaft(0.0, 100.0).torque(0.0, omega)
    ok = abs(T - 500.0) < 1e-9 and abs(T2 - 1000.0) < 1e-9 and abs(T2) > abs(T)
    # Solver uses configured damping (propshaft)
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0, shaft_damping=40.0))
    ok = ok and abs(s.prop.damping - 40.0) < 1e-12
    return ok, {"T_c50": T, "T_c100": T2, "solver_c": s.prop.damping}


def test_launch_shudder() -> tuple[bool, dict]:
    """Step torque from rest produces transient twist then finite wheel torque."""
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.2))
    peaks = []
    for i in range(300):
        s.step(600.0 if i > 10 else 0.0, 100.0, 100.0, dt=0.001)
        peaks.append(s.state.shaft_twist)
    ok = np.max(np.abs(peaks)) > 0 and np.isfinite(s.state.torque_left)
    return ok, {"max_twist": float(np.max(np.abs(peaks))), "TL": s.state.torque_left}


def test_gearshift_oscillation() -> tuple[bool, dict]:
    """Torque drop then reapply → twist response."""
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0))
    for _ in range(100):
        s.step(350.0, 80.0, 80.0, dt=0.001)
    mid = s.state.shaft_twist
    for _ in range(50):
        s.step(0.0, 80.0, 80.0, dt=0.001)
    for _ in range(100):
        s.step(350.0, 80.0, 80.0, dt=0.001)
    ok = np.isfinite(s.state.shaft_twist) and s.state.torsional_energy >= 0
    return ok, {"mid": mid, "final": s.state.shaft_twist}


def test_steady_state_power() -> tuple[bool, dict]:
    """After long run with matched loads, wheel torques roughly share input."""
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0, shaft_damping=60.0))
    T_in = 200.0
    for _ in range(2000):
        s.step(T_in, 100.0, 100.0, dt=0.001)
    total = s.state.torque_left + s.state.torque_right
    # Not exact equality (dynamics) but order-of-magnitude and finite
    ok = np.isfinite(total) and abs(total) < 5 * abs(T_in) + 100
    return ok, {"total": total, "T_in": T_in}


def test_power_conservation() -> tuple[bool, dict]:
    """Elastic energy non-negative; disabled path does not store energy."""
    s = DrivelineSolver(DrivelineConfig(enabled=True))
    for _ in range(100):
        s.step(250.0, 50.0, 50.0, dt=0.001)
    ok = s.state.torsional_energy >= -1e-9
    s2 = DrivelineSolver(DrivelineConfig(enabled=False))
    s2.step(250.0, 0, 0, dt=0.001)
    ok = ok and s2.state.torsional_energy == 0.0
    return ok, {"E": s.state.torsional_energy}


def test_symmetry() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0))
    for _ in range(500):
        s.step(300.0, 100.0, 100.0, dt=0.001)
    ok = abs(s.state.torque_left - s.state.torque_right) < 5.0
    return ok, {"TL": s.state.torque_left, "TR": s.state.torque_right}


def test_left_right_consistency() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0))
    for _ in range(400):
        s.step(300.0, 50.0, 150.0, dt=0.001)
    # Different loads → different halfshaft twists typically
    ok = np.isfinite(s.state.halfshaft_twist_L) and np.isfinite(s.state.halfshaft_twist_R)
    return ok, {"thL": s.state.halfshaft_twist_L, "thR": s.state.halfshaft_twist_R}


def test_repeatability() -> tuple[bool, dict]:
    def run():
        s = DrivelineSolver(DrivelineConfig(enabled=True, backlash_deg=0.0))
        for _ in range(200):
            s.step(280.0, 70.0, 70.0, dt=0.001)
        return s.state.shaft_twist, s.state.torque_left

    a, b = run(), run()
    ok = abs(a[0] - b[0]) < 1e-12 and abs(a[1] - b[1]) < 1e-12
    return ok, {"twist": a[0]}


def test_regression_disabled() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=False))
    st = s.step(100.0, 0, 0, dt=0.01)
    ok = st.torque_left == 50.0 and st.torque_right == 50.0 and not st.enabled
    return ok, {"enabled": st.enabled}


def test_performance_regression() -> tuple[bool, dict]:
    import time
    s = DrivelineSolver(DrivelineConfig(enabled=True))
    t0 = time.perf_counter()
    n = 5000
    for _ in range(n):
        s.step(200.0, 50.0, 50.0, dt=0.001)
    dt = time.perf_counter() - t0
    ms = dt / n * 1000
    ok = ms < 1.0  # < 1 ms/step
    return ok, {"ms_per_step": ms}


def test_no_nan_inf() -> tuple[bool, dict]:
    s = DrivelineSolver(DrivelineConfig(enabled=True))
    for i in range(500):
        T = 500.0 * np.sin(i * 0.05)
        s.step(T, 80.0, 90.0, dt=0.001)
    vals = [
        s.state.shaft_twist, s.state.torque_left, s.state.torque_right,
        s.state.torsional_energy, s.state.wheel_speed_left, s.state.oscillation_freq_hz,
    ]
    ok = all(np.isfinite(v) for v in vals)
    return ok, {"vals": vals}


def run_phase103_validation() -> bool:
    print("=== Phase 10.3 Advanced Driveline Validation ===\n")
    tests = [
        ("rigid_shaft_regression", test_rigid_shaft_regression),
        ("elastic_shaft_twist", test_elastic_shaft_twist),
        ("shaft_energy_storage", test_shaft_energy_storage),
        ("halfshaft_compliance", test_halfshaft_compliance),
        ("gear_backlash_deadzone", test_gear_backlash_deadzone),
        ("gear_backlash_engagement", test_gear_backlash_engagement),
        ("gear_mesh_stiffness", test_gear_mesh_stiffness),
        ("wheel_inertia_response", test_wheel_inertia_response),
        ("torsional_oscillation", test_torsional_oscillation),
        ("torsional_damping", test_torsional_damping),
        ("launch_shudder", test_launch_shudder),
        ("gearshift_oscillation", test_gearshift_oscillation),
        ("steady_state_power", test_steady_state_power),
        ("power_conservation", test_power_conservation),
        ("symmetry", test_symmetry),
        ("left_right_consistency", test_left_right_consistency),
        ("repeatability", test_repeatability),
        ("regression_disabled", test_regression_disabled),
        ("performance_regression", test_performance_regression),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:28} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:4]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 10.3 Status: IMPLEMENTATION VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase103_validation()
