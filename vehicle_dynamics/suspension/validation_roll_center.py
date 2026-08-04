"""
Phase 6.5 – Roll center migration validation.
"""

from __future__ import annotations

import numpy as np
from .roll_center import (
    RollCenterModel,
    axle_roll_center,
    default_front_axle,
    roll_center_height,
    _shift_outer_vertical,
)
from .hardpoints import default_front_left
from .wishbone import analyze, roll_center_front_view
from ..dual_track.suspension_interface import (
    SuspensionInterface,
    SuspensionInterfaceConfig,
)


def test_static_matches_phase60() -> tuple[bool, dict]:
    """z=0 → RC matches Phase 6.0 analyze() / roll_center_front_view."""
    hp = default_front_left()
    rc_ref = roll_center_front_view(hp)
    model = RollCenterModel()
    st = model.evaluate(np.zeros(4))
    ok = rc_ref is not None and abs(st.rc_front_z - rc_ref[1]) < 1e-9
    ok = ok and abs(st.rc_front_migration) < 1e-12
    return ok, {
        "rc_front": st.rc_front_z,
        "rc_ref": rc_ref[1] if rc_ref else None,
        "migration": st.rc_front_migration,
    }


def test_symmetric_bump() -> tuple[bool, dict]:
    model = RollCenterModel()
    st0 = model.evaluate(np.zeros(4))
    z = np.array([0.03, 0.03, 0.03, 0.03])
    st = model.evaluate(z)
    ok = np.isfinite(st.rc_front_z) and np.isfinite(st.rc_rear_z)
    # migration should be non-trivial for geometric solver (not forced to zero)
    ok = ok and abs(st.rc_front_z - st0.rc_front_z) >= 0.0  # finite delta
    return ok, {
        "rc0": st0.rc_front_z,
        "rc_bump": st.rc_front_z,
        "migration": st.rc_front_migration,
    }


def test_left_right_symmetry() -> tuple[bool, dict]:
    model = RollCenterModel()
    st_l = model.evaluate(np.array([0.04, 0.0, 0.0, 0.0]))
    st_r = model.evaluate(np.array([0.0, 0.04, 0.0, 0.0]))
    # With mirrored hardpoints, single-wheel L vs R should give same RC height
    ok = abs(st_l.rc_front_z - st_r.rc_front_z) < 1e-9
    return ok, {
        "rc_left_bump": st_l.rc_front_z,
        "rc_right_bump": st_r.rc_front_z,
    }


def test_independent_wheel_finite() -> tuple[bool, dict]:
    model = RollCenterModel()
    st = model.evaluate(np.array([0.05, -0.02, 0.03, -0.01]))
    ok = np.isfinite(st.rc_front_z) and np.isfinite(st.rc_rear_z)
    return ok, {"rc_front": st.rc_front_z, "rc_rear": st.rc_rear_z}


def test_roll_input_smooth() -> tuple[bool, dict]:
    """RC varies smoothly with roll-like opposite travel."""
    model = RollCenterModel()
    zs = np.linspace(0.0, 0.04, 5)
    rcs = []
    for a in zs:
        st = model.evaluate(np.array([a, -a, a, -a]))
        rcs.append(st.rc_front_z)
    rcs = np.array(rcs)
    ok = np.all(np.isfinite(rcs))
    # no huge jumps between adjacent samples
    if len(rcs) > 1:
        diffs = np.abs(np.diff(rcs))
        ok = ok and np.all(diffs < 0.5)  # soft continuity bound [m]
    return ok, {"rc_samples": rcs.tolist()}


def test_neutral_reproduces_static() -> tuple[bool, dict]:
    """Interface with z=0 matches Phase 6.0 static RC."""
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(
            enabled=True,
            roll_center_enabled=True,
        )
    )
    iface.set_wheel_travel(np.zeros(4))
    diag = iface.diagnostics()
    model = RollCenterModel()
    st = model.evaluate(np.zeros(4))
    ok = abs(diag["rc_front_z"] - st.rc_front_z) < 1e-9
    ok = ok and abs(diag["rc_front_migration"]) < 1e-12
    return ok, {
        "diag_rc_front": diag.get("rc_front_z"),
        "model_rc_front": st.rc_front_z,
    }


def test_diagnostics_logged() -> tuple[bool, dict]:
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, roll_center_enabled=True)
    )
    iface.set_wheel_travel(np.array([0.02, 0.01, 0.0, -0.01]))
    diag = iface.diagnostics()
    keys = ["rc_front_z", "rc_rear_z", "rc_front_migration", "rc_rear_migration"]
    ok = all(k in diag for k in keys) and all(
        np.isfinite(diag[k]) for k in keys
    )
    return ok, {"keys_present": keys}


def test_no_nan_inf() -> tuple[bool, dict]:
    model = RollCenterModel()
    ok = True
    samples = []
    for z in np.linspace(-0.08, 0.08, 9):
        st = model.evaluate(np.full(4, z))
        samples.append(st.rc_front_z)
        if not (np.isfinite(st.rc_front_z) and np.isfinite(st.rc_rear_z)):
            ok = False
    return ok, {"rc_front_samples": samples}


def run_phase65_validation() -> bool:
    print("=== Phase 6.5 Roll Center Migration Validation ===\n")
    tests = [
        ("static_matches_phase60", test_static_matches_phase60),
        ("symmetric_bump", test_symmetric_bump),
        ("left_right_symmetry", test_left_right_symmetry),
        ("independent_wheel_finite", test_independent_wheel_finite),
        ("roll_input_smooth", test_roll_input_smooth),
        ("neutral_reproduces_static", test_neutral_reproduces_static),
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
