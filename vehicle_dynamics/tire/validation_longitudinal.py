"""
Phase 3.3 – Pure Longitudinal Dugoff Validation

Committed, re-runnable checks for the claims historically recorded as:
  zero_slip, friction_limit, symmetry

These exercise the combined-slip API on the pure-longitudinal path (α = 0),
which is the Phase 3.3 behavioural contract preserved inside Phase 3.4.
"""

from __future__ import annotations

import numpy as np
from .dugoff import DugoffTire, DugoffParams


def test_zero_slip(tire: DugoffTire | None = None, Fz: float = 4000.0) -> tuple[bool, dict]:
    tire = tire or DugoffTire()
    s = tire.longitudinal_lateral_force(0.0, 0.0, Fz)
    ok = abs(s.Fx) < 1e-6 and abs(s.Fy) < 1e-6 and s.utilization < 1e-6
    return ok, {"Fx": float(s.Fx), "Fy": float(s.Fy), "utilization": float(s.utilization)}


def test_friction_limit(tire: DugoffTire | None = None, Fz: float = 4000.0) -> tuple[bool, dict]:
    tire = tire or DugoffTire()
    max_util = 0.0
    max_fx = 0.0
    for k in np.linspace(-1.0, 1.0, 401):
        s = tire.longitudinal_lateral_force(float(k), 0.0, Fz)
        max_util = max(max_util, s.utilization)
        max_fx = max(max_fx, abs(s.Fx))
        if abs(s.Fx) > tire.p.mu * Fz + 1e-2:
            return False, {"max_fx": float(abs(s.Fx)), "limit": tire.p.mu * Fz}
    ok = max_util <= 1.0 + 1e-6
    return ok, {"max_utilization": float(max_util), "max_fx": float(max_fx), "mu_Fz": tire.p.mu * Fz}


def test_symmetry(tire: DugoffTire | None = None, Fz: float = 4000.0) -> tuple[bool, dict]:
    """Fx(-κ) ≈ -Fx(+κ) on the pure-longitudinal path."""
    tire = tire or DugoffTire()
    max_asym = 0.0
    for k in [0.05, 0.10, 0.20, 0.40, 0.80]:
        pos = tire.longitudinal_lateral_force(k, 0.0, Fz)
        neg = tire.longitudinal_lateral_force(-k, 0.0, Fz)
        max_asym = max(max_asym, abs(pos.Fx + neg.Fx))
    ok = max_asym < 1.0
    return ok, {"max_asymmetry_N": float(max_asym)}


def test_linear_region(tire: DugoffTire | None = None, Fz: float = 4000.0) -> tuple[bool, dict]:
    """Near-zero numerical slope of Fx vs κ should track the Phase 3.3 linear form."""
    tire = tire or DugoffTire()
    # Fx0 = Cx * κ / (1+|κ|) → dFx/dκ at 0 = Cx
    k1, k2 = 1e-4, 5e-4
    f1 = tire.longitudinal_lateral_force(k1, 0.0, Fz).Fx
    f2 = tire.longitudinal_lateral_force(k2, 0.0, Fz).Fx
    slope = (f2 - f1) / (k2 - k1)
    # At very small κ, saturation factor ≈ 1, slope ≈ Cx
    rel_err = abs(slope - tire.p.Cx) / (tire.p.Cx + 1e-8)
    ok = rel_err < 0.10
    return ok, {"numerical_slope": float(slope), "expected_Cx": tire.p.Cx, "rel_error": float(rel_err)}


def run_longitudinal_validation(params: DugoffParams | None = None) -> bool:
    print("=== Phase 3.3 Pure-Longitudinal Dugoff Validation ===\n")
    tire = DugoffTire(params or DugoffParams())
    tests = [
        ("zero_slip", lambda: test_zero_slip(tire)),
        ("friction_limit", lambda: test_friction_limit(tire)),
        ("symmetry", lambda: test_symmetry(tire)),
        ("linear_region", lambda: test_linear_region(tire)),
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
    run_longitudinal_validation()
