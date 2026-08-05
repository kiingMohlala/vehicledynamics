"""Phase 10.0 – Powertrain foundation validation (target 15/15)."""

from __future__ import annotations

import numpy as np

from .engine import EngineConfig
from .engine_map import default_na_map
from .powertrain_solver import PowertrainSolver
from .rev_limiter import LimitMode


def test_zero_throttle_idle() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig(idle_rpm=900))
    for _ in range(300):
        st = s.step(throttle=0.0, load_torque=5.0, dt=0.01)
    ok = 700 < st.engine.rpm < 1200
    return ok, {"rpm": st.engine.rpm}


def test_idle_controller() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig(idle_rpm=900))
    # Disturb downward
    s.state.engine.rpm = 600
    s.state.engine.omega = 600 * 2 * np.pi / 60
    for _ in range(200):
        st = s.step(throttle=0.0, load_torque=8.0, dt=0.01)
    ok = st.engine.rpm > 750
    return ok, {"rpm": st.engine.rpm}


def test_torque_curve() -> tuple[bool, dict]:
    m = default_na_map(peak_torque=400, peak_torque_rpm=4500)
    tq_peak = m.torque_at(4500, 1.0)
    tq_low = m.torque_at(2000, 1.0)
    tq_high = m.torque_at(7000, 1.0)
    ok = tq_peak > tq_low and tq_peak > tq_high and tq_peak > 300
    return ok, {"peak": tq_peak, "low": tq_low, "high": tq_high}


def test_power_curve() -> tuple[bool, dict]:
    m = default_na_map()
    p4 = m.power_kw(4000, 1.0)
    p6 = m.power_kw(6000, 1.0)
    ok = p4 > 50 and p6 > p4 * 0.8
    return ok, {"p4": p4, "p6": p6}


def test_rpm_acceleration() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    s.state.engine.rpm = 2000
    s.state.engine.omega = 2000 * 2 * np.pi / 60
    r0 = s.state.engine.rpm
    for _ in range(100):
        st = s.step(throttle=1.0, load_torque=50.0, dt=0.01)
    ok = st.engine.rpm > r0 + 500
    return ok, {"rpm0": r0, "rpm1": st.engine.rpm}


def test_rpm_deceleration() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    s.state.engine.rpm = 5000
    s.state.engine.omega = 5000 * 2 * np.pi / 60
    r0 = s.state.engine.rpm
    for _ in range(150):
        st = s.step(throttle=0.0, load_torque=30.0, dt=0.01)
    ok = st.engine.rpm < r0 - 500
    return ok, {"rpm0": r0, "rpm1": st.engine.rpm}


def test_engine_braking() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    s.state.engine.rpm = 4000
    s.state.engine.omega = 4000 * 2 * np.pi / 60
    st = s.step(throttle=0.0, load_torque=0.0, dt=0.01)
    ok = st.engine.torque_output < 0 or st.engine.torque_brake < 0
    return ok, {"tq_out": st.engine.torque_output, "tq_eb": st.engine.torque_brake}


def test_rev_limiter() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig(redline_rpm=7500, soft_start_rpm=7300, limiter_mode=LimitMode.SOFT))
    s.state.engine.rpm = 7600
    s.state.engine.omega = 7600 * 2 * np.pi / 60
    st = s.step(throttle=1.0, load_torque=0.0, dt=0.01)
    ok = st.engine.limiter_factor < 0.1
    # Soft region
    s2 = PowertrainSolver(EngineConfig(redline_rpm=7500, soft_start_rpm=7300))
    s2.state.engine.rpm = 7400
    s2.state.engine.omega = 7400 * 2 * np.pi / 60
    st2 = s2.step(throttle=1.0, load_torque=0.0, dt=0.01)
    ok = ok and 0 < st2.engine.limiter_factor < 1
    return ok, {"lim_over": st.engine.limiter_factor, "lim_soft": st2.engine.limiter_factor}


def test_fuel_consumption() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    for _ in range(100):
        st = s.step(throttle=0.7, load_torque=100.0, dt=0.01)
    ok = st.fuel.fuel_total_g > 0 and st.fuel.fuel_rate_gps > 0
    return ok, {"total_g": st.fuel.fuel_total_g, "rate": st.fuel.fuel_rate_gps}


def test_thermal_warmup() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    t0 = s.state.thermal.coolant_C
    for _ in range(500):
        st = s.step(throttle=0.6, load_torque=120.0, dt=0.05)
    ok = st.thermal.coolant_C > t0 + 5
    ok = ok and st.thermal.efficiency_factor >= 0.8
    return ok, {"cool0": t0, "cool1": st.thermal.coolant_C, "eff": st.thermal.efficiency_factor}


def test_steady_state_power() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    # Balance roughly mid rpm
    s.state.engine.rpm = 4000
    s.state.engine.omega = 4000 * 2 * np.pi / 60
    # Find load that stabilizes
    for _ in range(50):
        st = s.step(throttle=1.0, load_torque=st.engine.torque_output if _ else 200.0, dt=0.01)
        load = st.engine.torque_output
        st = s.step(throttle=1.0, load_torque=load, dt=0.01)
    ok = st.engine.power_kw > 50
    return ok, {"power_kw": st.engine.power_kw, "rpm": st.engine.rpm}


def test_zero_speed_stability() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    s.state.engine.rpm = 0.0
    s.state.engine.omega = 0.0
    st = s.step(throttle=0.0, load_torque=0.0, dt=0.01)
    ok = st.engine.rpm >= 0 and np.isfinite(st.engine.torque_output)
    return ok, {"rpm": st.engine.rpm}


def test_regression_disabled() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig(enabled=False))
    st = s.step(throttle=1.0, load_torque=100.0, dt=0.01)
    ok = st.engine.torque_output == 0 and st.engine.rpm == 0
    return ok, {"tq": st.engine.torque_output}


def test_no_nan_inf() -> tuple[bool, dict]:
    s = PowertrainSolver(EngineConfig())
    ok = True
    for thr in (0.0, 0.3, 0.8, 1.0):
        for load in (0.0, 50.0, 200.0, 400.0):
            st = s.step(throttle=thr, load_torque=load, dt=0.01)
            vals = [st.engine.rpm, st.engine.torque_output, st.engine.power_kw,
                    st.fuel.fuel_rate_gps, st.thermal.coolant_C]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    """WOT from idle reaches significant power within 3 s unloaded."""
    s = PowertrainSolver(EngineConfig())
    peak_pwr = 0.0
    for _ in range(300):
        st = s.step(throttle=1.0, load_torque=20.0, dt=0.01)
        peak_pwr = max(peak_pwr, st.engine.power_kw)
    ok = st.engine.rpm > 3000 and peak_pwr > 40
    return ok, {"rpm": st.engine.rpm, "power": st.engine.power_kw, "peak_power": peak_pwr}


def run_phase100_validation() -> bool:
    print("=== Phase 10.0 Powertrain Foundation Validation ===\n")
    tests = [
        ("zero_throttle_idle", test_zero_throttle_idle),
        ("idle_controller", test_idle_controller),
        ("torque_curve", test_torque_curve),
        ("power_curve", test_power_curve),
        ("rpm_acceleration", test_rpm_acceleration),
        ("rpm_deceleration", test_rpm_deceleration),
        ("engine_braking", test_engine_braking),
        ("rev_limiter", test_rev_limiter),
        ("fuel_consumption", test_fuel_consumption),
        ("thermal_warmup", test_thermal_warmup),
        ("steady_state_power", test_steady_state_power),
        ("zero_speed_stability", test_zero_speed_stability),
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
        print("Phase 10.0 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase100_validation()
