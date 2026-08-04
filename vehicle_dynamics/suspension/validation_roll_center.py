"""
Phase 6.5 – Roll center migration validation.
"""

from __future__ import annotations

import numpy as np
from .roll_center import (
    compute_roll_centers,
    RollCenterGeometry,
    RollCenterModel,
    corner_roll_center_z,
    displace_corner,
)
from .hardpoints import default_front_left
from .wishbone import roll_center_front_view
from ..dual_track.suspension_interface import (
    SuspensionInterface,
    SuspensionInterfaceConfig,
)


def test_static_matches_phase60() -> tuple[bool, dict]:
    """z=0 → same RC as Phase 6.0 analyze/roll_center_front_view."""
    hp = default_front_left()
    rc_phase60 = roll_center_front_view(hp)
    st = compute_roll_centers(np.zeros(4))
    ok = rc_phase60 is not None and abs(st.rc_front - rc_phase60[1]) < 1e-9
    ok = ok and abs(st.rc_front_migration) < 1e-12
    ok = ok and abs(st.rc_rear_migration) < 1e-12
    return ok, {
        "rc_front": st.rc_front,
        "phase60_rc_z": rc_phase60[1] if rc_phase60 else None,
        "migration": st.rc_front_migration,
    }


def test_symmetric_bump() -> tuple[bool, dict]:
    z = np.array([0.03, 0.03, 0.02, 0.02])
    st = compute_roll_centers(z)
    ok = np.isfinite(st.rc_front) and np.isfinite(st.rc_rear)
    # left/right IC should be mirrors in magnitude for y when travel equal
    ok = ok and abs(st.ic_y[0] + st.ic_y[1]) < 1e-6  # opposite y signs
    return ok, {
        "rc_front": st.rc_front,
        "rc_rear": st.rc_rear,
        "migration_f": st.rc_front_migration,
        "ic_y": st.ic_y.tolist(),
    }


def test_left_right_symmetry() -> tuple[bool, dict]:
    st = compute_roll_centers(np.array([0.04, 0.04, 0.0, 0.0]))
    ok = abs(st.ic_y[0] + st.ic_y[1]) < 1e-6 and abs(st.ic_z[0] - st.ic_z[1]) < 1e-6
    return ok, {"ic_y": st.ic_y[:2].tolist(), "ic_z": st.ic_z[:2].tolist()}


def test_independent_wheel_bump_finite() -> tuple[bool, dict]:
    st = compute_roll_centers(np.array([0.05, 0.0, 0.0, 0.0]))
    ok = np.isfinite(st.rc_front) and np.isfinite(st.rc_rear)
    ok = ok and np.isfinite(st.ic_y[0]) and np.isfinite(st.ic_z[0])
    return ok, {
        "rc_front": st.rc_front,
        "migration": st.rc_front_migration,
        "ic_fl": (float(st.ic_y[0]), float(st.ic_z[0])),
    }


def test_roll_input_smooth() -> tuple[bool, dict]:
    """Opposite left/right travel (roll) → finite smooth RC change."""
    heights = []
    for a in np.linspace(0.0, 0.04, 5):
        st = compute_roll_centers(np.array([a, -a, 0.0, 0.0]))
        heights.append(st.rc_front)
    heights = np.array(heights)
    ok = np.all(np.isfinite(heights))
    # no huge jumps between adjacent samples
    diffs = np.abs(np.diff(heights))
    ok = ok and (len(diffs) == 0 or np.max(diffs) < 0.5)
    return ok, {"rc_front_series": heights.tolist(), "max_step": float(np.max(diffs)) if len(diffs) else 0.0}


def test_neutral_reproduces_phase64_steer() -> tuple[bool, dict]:
    """Roll-center path must not alter effective_steer (diagnostic only)."""
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(
            enabled=True,
            bump_steer_enabled=False,
            camber_gain_enabled=False,
            roll_center_enabled=True,
        )
    )
    z = np.array([0.03, 0.03, 0.02, 0.02])
    d0 = iface.effective_steer(0.1, 0.1, wheel_travel=np.zeros(4))
    d1 = iface.effective_steer(0.1, 0.1, wheel_travel=z)
    ok = bool(np.allclose(d0, d1))
    return ok, {"d0": d0.tolist(), "d1": d1.tolist()}


def test_diagnostics_logged() -> tuple[bool, dict]:
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, roll_center_enabled=True)
    )
    iface.set_wheel_travel(np.array([0.02, 0.02, 0.01, 0.01]))
    diag = iface.diagnostics()
    ok = (
        "rc_front_m" in diag
        and "rc_rear_m" in diag
        and "rc_front_migration_m" in diag
        and np.isfinite(diag["rc_front_m"])
    )
    return ok, {"rc_front": diag.get("rc_front_m"), "migration": diag.get("rc_front_migration_m")}


def test_no_nan_inf() -> tuple[bool, dict]:
    travels = [
        np.zeros(4),
        np.array([0.05, 0.05, 0.05, 0.05]),
        np.array([0.05, -0.03, 0.02, -0.01]),
        np.array([-0.04, -0.04, 0.03, 0.03]),
    ]
    ok = True
    vals = []
    for z in travels:
        st = compute_roll_centers(z)
        vals.append((st.rc_front, st.rc_rear))
        if not (np.isfinite(st.rc_front) and np.isfinite(st.rc_rear)):
            ok = False
    return ok, {"samples": vals}


def run_phase65_validation() -> bool:
    print("=== Phase 6.5 Roll Center Migration Validation ===\n")
    tests = [
        ("static_matches_phase60", test_static_matches_phase60),
        ("symmetric_bump", test_symmetric_bump),
        ("left_right_symmetry", test_left_right_symmetry),
        ("independent_wheel_bump_finite", test_independent_wheel_bump_finite),
        ("roll_input_smooth", test_roll_input_smooth),
        ("neutral_reproduces_phase64_steer", test_neutral_reproduces_phase64_steer),
        ("diagnostics_logged", test_diagnostics_logged),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase65_validation()
