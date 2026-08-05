"""Phase 10.1 – Clutch & gearbox validation (target 16/16)."""

from __future__ import annotations

import numpy as np

from vehicle_dynamics.powertrain.engine import EngineState

from .gear_ratios import default_ratios
from .clutch_friction import clutch_capacity, ClutchFrictionParams
from .transmission_solver import TransmissionConfig, TransmissionSolver
from .shift_controller import ShiftPhase


def _eng(rpm=3000.0, tq=200.0) -> EngineState:
    omega = rpm * 2 * np.pi / 60
    return EngineState(rpm=rpm, omega=omega, torque_output=tq, torque_indicated=tq)


def test_neutral_zero_output() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=0))
    st = s.step(_eng(), clutch=1.0, requested_gear=0, dt=0.01)
    ok = abs(st.wheel_torque) < 1e-6 and st.gear == 0
    return ok, {"gear": st.gear, "wtq": st.wheel_torque}


def test_clutch_fully_engaged() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=3))
    # Spin wheels so gb matches engine approximately
    ratios = default_ratios()
    omega_w = _eng(3000, 200).omega / ratios.overall(3)
    for _ in range(50):
        st = s.step(_eng(3000, 200), clutch=1.0, requested_gear=3, dt=0.01, omega_wheel=omega_w)
    ok = st.locked or abs(st.clutch_slip) < 5.0
    ok = ok and abs(st.wheel_torque) > 100
    return ok, {"slip": st.clutch_slip, "wtq": st.wheel_torque, "locked": st.locked}


def test_clutch_slip() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=1))
    s.shift.state.current_gear = 1
    s.gearbox.current_gear = 1
    # requested_gear=0 holds gear in sequential mode (avoid +1 upshift)
    st = s.step(_eng(4000, 250), clutch=0.3, requested_gear=0, dt=0.01, omega_wheel=0.0)
    ok = abs(st.clutch_slip) > 1.0 and abs(st.clutch_torque) > 0
    return ok, {"slip": st.clutch_slip, "ctq": st.clutch_torque, "engage": st.clutch_engagement}


def test_gear_ratio_output() -> tuple[bool, dict]:
    ratios = default_ratios(3.9)
    t_in = 100.0
    t_out = ratios.output_torque(t_in, 2)
    expected = 100.0 * ratios.gears[2] * ratios.primary * 3.9 * ratios.efficiency
    ok = abs(t_out - expected) < 1e-6
    return ok, {"t_out": t_out, "expected": expected}


def test_reverse_gear() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=0, gearbox="manual"))
    for _ in range(100):
        st = s.step(_eng(1500, 80), clutch=0.0 if s.shift.state.in_progress else 1.0,
                    requested_gear=-1, dt=0.02, omega_wheel=0.0)
    for _ in range(50):
        st = s.step(_eng(1500, 80), clutch=1.0, requested_gear=-1, dt=0.01, omega_wheel=0.0)
    ok = st.gear == -1 and abs(st.wheel_torque) > 50
    return ok, {"gear": st.gear, "wtq": st.wheel_torque, "ctq": st.clutch_torque, "phase": st.shift_phase}


def test_sequential_shift() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=2, gearbox="sequential"))
    s.shift.state.current_gear = 2
    s.gearbox.current_gear = 2
    # Upshift request +1
    s.shift.request(3)
    for _ in range(100):
        st = s.step(_eng(4000, 150), clutch=1.0, requested_gear=0, dt=0.01)
    ok = st.gear == 3 and not st.shift_active
    return ok, {"gear": st.gear, "phase": st.shift_phase}


def test_manual_shift() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=1, gearbox="manual"))
    s.shift.state.current_gear = 1
    s.gearbox.current_gear = 1
    s.shift.request(4)
    for _ in range(100):
        st = s.step(_eng(3500, 120), clutch=0.2, requested_gear=4, dt=0.01)
    ok = st.gear == 4
    return ok, {"gear": st.gear}


def test_shift_delay() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=2))
    s.shift.state.current_gear = 2
    s.gearbox.current_gear = 2
    s.shift.request(3)
    phases = []
    for _ in range(60):
        st = s.step(_eng(4000, 100), clutch=1.0, requested_gear=0, dt=0.01)
        phases.append(st.shift_phase)
    ok = "cut" in phases and "disengage" in phases and ("sync" in phases or "engage" in phases)
    return ok, {"phases": list(dict.fromkeys(phases))}


def test_rpm_matching() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=2))
    s.shift.state.current_gear = 2
    s.shift.request(3)
    saw_sync = False
    for _ in range(80):
        st = s.step(_eng(5000, 100), clutch=1.0, requested_gear=0, dt=0.01)
        if st.shift_phase == "sync":
            saw_sync = True
    ok = saw_sync or st.gear == 3
    return ok, {"saw_sync": saw_sync, "gear": st.gear}


def test_launch_control() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=1))
    s.shift.state.current_gear = 1
    s.gearbox.current_gear = 1
    st = s.step(_eng(4500, 220), clutch=1.0, requested_gear=1, dt=0.01, launch=True)
    ok = st.clutch_engagement < 1.0  # launch limits engagement
    return ok, {"engage": st.clutch_engagement}


def test_power_flow() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=2))
    ratios = default_ratios()
    omega_w = 3000 * 2 * np.pi / 60 / ratios.overall(2)
    for _ in range(30):
        st = s.step(_eng(3000, 150), clutch=1.0, requested_gear=2, dt=0.01, omega_wheel=omega_w)
    ok = st.wheel_torque > st.clutch_torque > 0  # multiplication
    return ok, {"ctq": st.clutch_torque, "wtq": st.wheel_torque}


def test_torque_conservation() -> tuple[bool, dict]:
    ratios = default_ratios(3.9)
    t_in = 50.0
    for g in range(1, 6):
        t_out = ratios.output_torque(t_in, g)
        # Power conserved aside from efficiency: T_out * w_out ≈ η T_in * w_in
        w_in = 100.0
        w_out = ratios.output_omega(w_in, g)
        p_in = t_in * w_in
        p_out = t_out * w_out
        if abs(p_in) > 1e-9:
            eta = p_out / p_in
            if abs(eta - ratios.efficiency) > 1e-6:
                return False, {"gear": g, "eta": eta}
    return True, {"eta": ratios.efficiency}


def test_thermal_accumulation() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=1))
    s.shift.state.current_gear = 1
    s.gearbox.current_gear = 1
    t0 = s.clutch.state.temp_C
    for _ in range(100):
        st = s.step(_eng(5000, 250), clutch=0.4, requested_gear=1, dt=0.02, omega_wheel=0.0)
    ok = st.clutch_temp_C > t0
    return ok, {"t0": t0, "t1": st.clutch_temp_C}


def test_regression_disabled() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(enabled=False))
    st = s.step(_eng(4000, 200), clutch=1.0, requested_gear=3, dt=0.01)
    ok = st.wheel_torque == 0 and st.gear == 0
    return ok, {"wtq": st.wheel_torque}


def test_no_nan_inf() -> tuple[bool, dict]:
    s = TransmissionSolver(TransmissionConfig(initial_gear=1))
    ok = True
    for g in (0, 1, 3, -1):
        for c in (0.0, 0.5, 1.0):
            st = s.step(_eng(2500, 100), clutch=c, requested_gear=g, dt=0.01)
            vals = [st.wheel_torque, st.clutch_slip, st.gearbox_rpm, st.clutch_temp_C]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    """1st gear launch produces large wheel torque."""
    s = TransmissionSolver(TransmissionConfig(initial_gear=1))
    s.shift.state.current_gear = 1
    s.gearbox.current_gear = 1
    peak = 0.0
    for _ in range(50):
        st = s.step(_eng(4000, 280), clutch=0.85, requested_gear=1, dt=0.01, omega_wheel=0.0)
        peak = max(peak, abs(st.wheel_torque))
    ok = peak > 500
    return ok, {"peak_wtq": peak}


def run_phase101_validation() -> bool:
    print("=== Phase 10.1 Clutch & Gearbox Validation ===\n")
    tests = [
        ("neutral_zero_output", test_neutral_zero_output),
        ("clutch_fully_engaged", test_clutch_fully_engaged),
        ("clutch_slip", test_clutch_slip),
        ("gear_ratio_output", test_gear_ratio_output),
        ("reverse_gear", test_reverse_gear),
        ("sequential_shift", test_sequential_shift),
        ("manual_shift", test_manual_shift),
        ("shift_delay", test_shift_delay),
        ("rpm_matching", test_rpm_matching),
        ("launch_control", test_launch_control),
        ("power_flow", test_power_flow),
        ("torque_conservation", test_torque_conservation),
        ("thermal_accumulation", test_thermal_accumulation),
        ("regression_disabled", test_regression_disabled),
        ("no_nan_inf", test_no_nan_inf),
        ("performance_regression", test_performance_regression),
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
        print("Phase 10.1 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase101_validation()
