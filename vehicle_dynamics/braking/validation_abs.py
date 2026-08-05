"""
Phase 3.2 – ABS Controller Unit Validation

Committed, re-runnable checks for the claims historically recorded as:
  pressure_bounds, pressure_modulation, state_machine

These tests exercise ABSController in isolation (no vehicle model required).
"""

from __future__ import annotations

import numpy as np
from .abs_controller import ABSController, ABSParams


def test_pressure_bounds(dt: float = 0.001, n_steps: int = 2000) -> tuple[bool, dict]:
    """Pressure must remain in [min_pressure, 1.0] for all slip inputs."""
    ctrl = ABSController()
    pressures = []
    # Sweep slips from locked to free-rolling
    slips = np.concatenate([
        np.linspace(0.0, 0.5, n_steps // 2),
        np.linspace(0.5, 0.0, n_steps // 2),
    ])
    for s in slips:
        p = ctrl.update(float(s), dt)
        pressures.append(p)

    pressures = np.asarray(pressures)
    lo = ctrl.p.min_pressure
    ok = bool(np.all(pressures >= lo - 1e-9) and np.all(pressures <= 1.0 + 1e-9))
    return ok, {
        "min_pressure": float(pressures.min()),
        "max_pressure": float(pressures.max()),
        "allowed_lo": lo,
        "allowed_hi": 1.0,
    }


def test_pressure_modulation(dt: float = 0.001) -> tuple[bool, dict]:
    """High slip must reduce pressure; low slip must allow rebuild."""
    ctrl = ABSController()
    pressures = []
    states = []

    # Force release: sustained high slip
    for _ in range(250):
        pressures.append(ctrl.update(0.35, dt))
        states.append(ctrl.state)

    # Allow rebuild: sustained low slip
    for _ in range(400):
        pressures.append(ctrl.update(0.08, dt))
        states.append(ctrl.state)

    # Second lock event
    for _ in range(200):
        pressures.append(ctrl.update(0.30, dt))
        states.append(ctrl.state)

    pressures = np.asarray(pressures)
    ptp = float(np.ptp(pressures))
    ok_bounds = bool(np.all(pressures >= ctrl.p.min_pressure - 1e-9) and np.all(pressures <= 1.0 + 1e-9))
    ok_mod = ptp > 0.10
    ok_states = ("release" in states) and ("build" in states)
    ok = ok_bounds and ok_mod and ok_states
    return ok, {
        "pressure_min": float(pressures.min()),
        "pressure_max": float(pressures.max()),
        "pressure_ptp": ptp,
        "states_seen": sorted(set(states)),
    }


def test_state_machine(dt: float = 0.001) -> tuple[bool, dict]:
    """FSM must visit build → release → hold (or build) under a lock/recovery cycle."""
    ctrl = ABSController()
    transitions = []
    prev = ctrl.state

    sequence = (
        [(0.05, 50), (0.30, 200), (0.18, 150), (0.08, 200), (0.28, 150)]
    )
    for slip, n in sequence:
        for _ in range(n):
            ctrl.update(slip, dt)
            if ctrl.state != prev:
                transitions.append((prev, ctrl.state))
                prev = ctrl.state

    states_visited = {prev}
    for a, b in transitions:
        states_visited.add(a)
        states_visited.add(b)

    has_release = any(b == "release" for _, b in transitions) or "release" in states_visited
    has_build = "build" in states_visited
    # hold is optional depending on timing, but typical
    ok = has_release and has_build and len(transitions) >= 1
    return ok, {
        "transitions": transitions[:20],
        "states_visited": sorted(states_visited),
        "n_transitions": len(transitions),
    }


def run_abs_validation() -> bool:
    print("=== Phase 3.2 ABS Controller Validation ===\n")
    tests = [
        ("pressure_bounds", test_pressure_bounds),
        ("pressure_modulation", test_pressure_modulation),
        ("state_machine", test_state_machine),
    ]
    all_pass = True
    results = {}
    for name, fn in tests:
        ok, diag = fn()
        results[name] = ok
        print(f"{name:28} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_abs_validation()
