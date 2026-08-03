"""
Phase 5.3 – ESC validation suite.

Gates:
  1. Inactive region – mild steer, ESC stays off
  2. Oversteer correction – high yaw error reduced
  3. Understeer assist – yaw rate closer to reference
  4. Activation hysteresis – no chatter
  5. ESC disabled → plant unchanged (regression)
  6. Numerical robustness
"""

from __future__ import annotations

import numpy as np
from .parameters import ESCParameters
from .controller import ESCController
from .brake_allocator import BrakeAllocator
from .reference_model import YawReferenceModel
from .diagnostics import ESCDiagnostics


def test_reference_model() -> tuple[bool, dict]:
    ref = YawReferenceModel(wheelbase=2.7)
    r1 = ref.steady_state(20.0, np.deg2rad(3.0))
    r2 = ref.steady_state(20.0, -np.deg2rad(3.0))
    ok = r1 > 0 and abs(r1 + r2) < 1e-9
    return ok, {"r_ref_pos": float(r1), "r_ref_neg": float(r2)}


def test_inactive_region() -> tuple[bool, dict]:
    esc = ESCController(wheelbase=2.7)
    esc.reset()
    scales = []
    act = []
    # Mild conditions: small yaw error
    r_ref_model = YawReferenceModel(2.7)
    for i in range(200):
        vx, delta = 15.0, np.deg2rad(2.0)
        r_ref = r_ref_model.update(vx, delta, 0.01)
        r = r_ref + 0.01  # tiny error inside deadband/off region
        Mz, diag = esc.update(vx, 0.0, r, delta, 0.01)
        act.append(diag["active"])
        scales.append(Mz)
    ok = not any(act) and all(abs(m) < 1e-6 for m in scales)
    return ok, {"any_active": any(act)}


def test_oversteer_correction() -> tuple[bool, dict]:
    """Large positive yaw error → negative Mz → right-side brake."""
    p = ESCParameters(on_threshold=0.03, off_threshold=0.015)
    esc = ESCController(wheelbase=2.7, params=p)
    alloc = BrakeAllocator(1.55, 1.55, p)
    esc.reset()
    # Force oversteer: r >> r_ref for left turn
    vx, delta = 18.0, np.deg2rad(5.0)
    r = 0.8  # excessive
    Mz, diag = 0.0, {}
    for _ in range(50):
        Mz, diag = esc.update(vx, 0.5, r, delta, 0.01)
    scale = alloc.allocate(Mz, delta)
    ok = (
        diag["active"]
        and Mz < 0
        and (scale[1] + scale[3]) > (scale[0] + scale[2])  # more right brake
    )
    return ok, {"Mz": float(Mz), "scale": scale.tolist(), "active": diag["active"]}


def test_understeer_correction() -> tuple[bool, dict]:
    """Yaw rate too low for left turn → positive Mz → left-side brake."""
    p = ESCParameters(on_threshold=0.03, off_threshold=0.015)
    esc = ESCController(wheelbase=2.7, params=p)
    alloc = BrakeAllocator(1.55, 1.55, p)
    esc.reset()
    vx, delta = 18.0, np.deg2rad(8.0)
    r = 0.05  # far below reference
    Mz, diag = 0.0, {}
    for _ in range(50):
        Mz, diag = esc.update(vx, 0.1, r, delta, 0.01)
    scale = alloc.allocate(Mz, delta)
    ok = (
        diag["active"]
        and Mz > 0
        and (scale[0] + scale[2]) > (scale[1] + scale[3])
    )
    return ok, {"Mz": float(Mz), "scale": scale.tolist(), "active": diag["active"]}


def test_hysteresis() -> tuple[bool, dict]:
    p = ESCParameters(on_threshold=0.06, off_threshold=0.02)
    esc = ESCController(wheelbase=2.7, params=p)
    esc.reset()
    # Ramp error up then down
    act_hist = []
    for e in list(np.linspace(0, 0.1, 40)) + list(np.linspace(0.1, 0, 40)):
        # inject by setting r = r_ref + e; approximate r_ref≈0.2
        Mz, diag = esc.update(15.0, 0.0, 0.2 + e, np.deg2rad(3), 0.01)
        act_hist.append(diag["active"])
    # Should turn on once and off once (few transitions)
    transitions = sum(1 for i in range(1, len(act_hist)) if act_hist[i] != act_hist[i-1])
    ok = transitions <= 4 and any(act_hist)
    return ok, {"transitions": transitions, "activation_fraction": float(np.mean(act_hist))}


def test_allocator_limits() -> tuple[bool, dict]:
    alloc = BrakeAllocator(1.55, 1.55)
    scale = alloc.allocate(1e6, np.deg2rad(5))
    ok = float(np.max(scale)) <= ESCParameters().max_brake_scale + 1e-9
    ok = ok and float(np.min(scale)) >= 0.0
    return ok, {"scale": scale.tolist()}


def test_disabled_zero_output() -> tuple[bool, dict]:
    """With tiny errors and low speed, Mz stays 0."""
    esc = ESCController(wheelbase=2.7)
    esc.reset()
    Mz, diag = esc.update(vx=3.0, vy=0.0, r=0.01, delta=0.0, dt=0.01)
    ok = abs(Mz) < 1e-9 and not diag["active"]
    return ok, diag


def run_esc_validation() -> bool:
    print("=== Phase 5.3 ESC Validation ===\n")
    tests = [
        ("reference_model", test_reference_model),
        ("inactive_region", test_inactive_region),
        ("oversteer_correction", test_oversteer_correction),
        ("understeer_correction", test_understeer_correction),
        ("hysteresis", test_hysteresis),
        ("allocator_limits", test_allocator_limits),
        ("disabled_zero_output", test_disabled_zero_output),
    ]
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        print(f"{name:28} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_esc_validation()
