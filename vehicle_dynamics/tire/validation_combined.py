"""
Phase 3.4.2 – Combined-Slip Dugoff Validation Suite

Improvements:
1. True regression against validated Phase 3.3 longitudinal model
2. Pure lateral linear-slope check (dFy/dα ≈ Cy)
3. Reciprocal combined-slip coupling
4. Surface continuity with numerical gradients
5. Clamp activation statistics + warning threshold
"""

from dataclasses import dataclass, field
import numpy as np
from .dugoff import DugoffTire, DugoffParams, TireState

@dataclass
class ValidationResult:
    passed: bool
    tests: dict = field(default_factory=dict)
    diagnostics: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)


# ------------------------------------------------------------------
# Helper: pure longitudinal reference (Phase 3.3 behaviour)
# ------------------------------------------------------------------
class StandardDugoffLongitudinal:
    """Reference pure-longitudinal model matching Phase 3.3"""
    def __init__(self, params: DugoffParams):
        self.p = params

    def longitudinal_force(self, slip_ratio: float, Fz: float) -> TireState:
        kappa = np.clip(slip_ratio, -1.0, 1.0)
        Fz = max(Fz, 1.0)

        if abs(kappa) < 1e-8:
            return TireState(
                Fx=0.0, Fy=0.0, Fx_linear=0.0, Fy_linear=0.0,
                slip_ratio=kappa, slip_angle=0.0,
                lambda_=1.0, utilization=0.0, saturated=False
            )

        Fx0 = self.p.Cx * kappa / (1.0 + abs(kappa))
        lambda_ = (self.p.mu * Fz * (1.0 + abs(kappa))) / (2.0 * self.p.Cx * abs(kappa) + 1e-8)

        if lambda_ >= 1.0:
            f = 1.0
            saturated = False
        else:
            f = lambda_ * (2.0 - lambda_)
            saturated = True

        Fx = Fx0 * f
        Fx = np.clip(Fx, -self.p.mu * Fz, self.p.mu * Fz)
        utilization = abs(Fx) / (self.p.mu * Fz + 1e-8)

        return TireState(
            Fx=Fx, Fy=0.0, Fx_linear=Fx0, Fy_linear=0.0,
            slip_ratio=kappa, slip_angle=0.0,
            lambda_=lambda_, utilization=utilization, saturated=saturated
        )


# ------------------------------------------------------------------
# 1. Phase 3.3 Compatibility (true regression)
# ------------------------------------------------------------------
def test_phase33_compatibility(tire: DugoffTire, Fz: float = 4000.0, tol: float = 0.02):
    ref_tire = StandardDugoffLongitudinal(tire.p)
    kappas = np.linspace(-1.0, 1.0, 201)
    abs_errors = []

    for k in kappas:
        ref = ref_tire.longitudinal_force(k, Fz)
        new = tire.longitudinal_lateral_force(k, 0.0, Fz)
        abs_errors.append(abs(new.Fx - ref.Fx))

    rms = np.sqrt(np.mean(np.array(abs_errors)**2))
    max_err = np.max(abs_errors)
    passed = (rms < tol * Fz) and (max_err < 2 * tol * Fz)

    return passed, {
        "rms_error": float(rms),
        "max_error": float(max_err),
        "tolerance": tol * Fz
    }


# ------------------------------------------------------------------
# 2. Pure Lateral Validation
# ------------------------------------------------------------------
def test_pure_lateral(tire: DugoffTire, Fz: float = 4000.0):
    alphas = np.linspace(-np.deg2rad(15), np.deg2rad(15), 61)
    Fx_ok = True
    symmetry_ok = True

    for a in alphas:
        state = tire.longitudinal_lateral_force(0.0, a, Fz)
        if abs(state.Fx) > 1.0:          # allow tiny numerical noise
            Fx_ok = False
        state_neg = tire.longitudinal_lateral_force(0.0, -a, Fz)
        if abs(state.Fy + state_neg.Fy) > 1.0:
            symmetry_ok = False

    # Linear slope check near zero
    s1, s2 = 0.001, 0.005
    Fy1 = tire.longitudinal_lateral_force(0.0, s1, Fz).Fy
    Fy2 = tire.longitudinal_lateral_force(0.0, s2, Fz).Fy
    numerical_slope = (Fy2 - Fy1) / (s2 - s1)
    slope_error = abs(numerical_slope - tire.p.Cy) / (tire.p.Cy + 1e-8)
    slope_ok = slope_error < 0.08

    return Fx_ok and symmetry_ok and slope_ok, {
        "numerical_slope": float(numerical_slope),
        "expected_Cy": tire.p.Cy,
        "slope_error": float(slope_error)
    }


# ------------------------------------------------------------------
# 3. Combined-Slip Coupling (both directions)
# ------------------------------------------------------------------
def test_combined_coupling(tire: DugoffTire, Fz: float = 4000.0):
    kappas = [0.0, 0.05, 0.10, 0.20, 0.30]
    alphas = np.deg2rad([0, 5, 10])

    # Increasing |κ| should reduce available |Fy|
    fy_ok = True
    for a in alphas:
        if abs(a) < 1e-6:
            continue
        fy_prev = None
        for k in kappas:
            state = tire.longitudinal_lateral_force(k, a, Fz)
            if fy_prev is not None:
                if abs(state.Fy) > abs(fy_prev) + 10.0:
                    fy_ok = False
            fy_prev = state.Fy

    # Reciprocal: increasing |α| should reduce available |Fx|
    fx_ok = True
    alphas_sweep = np.deg2rad([0, 5, 10, 15])
    for k in [0.15, 0.25]:
        fx_prev = None
        for a in alphas_sweep:
            state = tire.longitudinal_lateral_force(k, a, Fz)
            if fx_prev is not None and abs(a) > 1e-4:
                if abs(state.Fx) > abs(fx_prev) + 10.0:
                    fx_ok = False
            fx_prev = state.Fx

    return fy_ok and fx_ok


# ------------------------------------------------------------------
# 4. Friction limit + Clamp statistics
# ------------------------------------------------------------------
def test_friction_and_clamp(tire: DugoffTire, Fz: float = 4000.0):
    kappas = np.linspace(-1.0, 1.0, 41)
    alphas = np.deg2rad(np.linspace(-15, 15, 31))
    clamp_count = 0
    total = 0
    max_util = 0.0
    min_scale = 1.0

    for k in kappas:
        for a in alphas:
            state = tire.longitudinal_lateral_force(k, a, Fz)
            total += 1
            if state.clamp_activated:
                clamp_count += 1
                min_scale = min(min_scale, state.clamp_scale)
            max_util = max(max_util, state.utilization)

            if np.sqrt(state.Fx**2 + state.Fy**2) > tire.p.mu * Fz + 1e-2:
                return False, {
                    "clamp_activations": clamp_count,
                    "max_utilization": max_util
                }

    activation_pct = 100.0 * clamp_count / total
    return True, {
        "total_evaluations": total,
        "clamp_activations": clamp_count,
        "activation_pct": activation_pct,
        "max_utilization": max_util,
        "min_clamp_scale": min_scale
    }


# ------------------------------------------------------------------
# 5. Surface Continuity + Gradients
# ------------------------------------------------------------------
def test_surface_continuity(tire: DugoffTire, Fz: float = 4000.0):
    kappas = np.linspace(-1.0, 1.0, 41)
    alphas = np.deg2rad(np.linspace(-15, 15, 31))

    Fx_grid = np.zeros((len(kappas), len(alphas)))
    Fy_grid = np.zeros_like(Fx_grid)

    for i, k in enumerate(kappas):
        for j, a in enumerate(alphas):
            state = tire.longitudinal_lateral_force(k, a, Fz)
            if not np.isfinite([state.Fx, state.Fy, state.lambda_, state.utilization]).all():
                return False
            Fx_grid[i, j] = state.Fx
            Fy_grid[i, j] = state.Fy

    # Numerical gradients
    dFx_dk = np.gradient(Fx_grid, kappas, axis=0)
    dFy_da = np.gradient(Fy_grid, alphas, axis=1)

    if not (np.all(np.isfinite(dFx_dk)) and np.all(np.isfinite(dFy_da))):
        return False

    return True


# ------------------------------------------------------------------
# Main runner
# ------------------------------------------------------------------
def run_all_combined_tests(params: DugoffParams = None) -> ValidationResult:
    if params is None:
        params = DugoffParams()
    tire = DugoffTire(params)
    Fz = 4000.0

    tests = {}
    diagnostics = {}
    warnings = []

    # 1. Phase 3.3 compatibility
    ok, diag = test_phase33_compatibility(tire, Fz)
    tests["phase33_compatibility"] = ok
    diagnostics.update({f"compat_{k}": v for k, v in diag.items()})

    # 2. Pure lateral
    ok, diag = test_pure_lateral(tire, Fz)
    tests["pure_lateral"] = ok
    diagnostics.update({f"lateral_{k}": v for k, v in diag.items()})

    # 3. Combined coupling
    tests["combined_coupling"] = test_combined_coupling(tire, Fz)

    # 4. Friction + clamp
    ok, diag = test_friction_and_clamp(tire, Fz)
    tests["friction_limit"] = ok
    diagnostics.update(diag)

    if diag.get("activation_pct", 0) > 1.0:
        warnings.append(
            f"Clamp activation {diag['activation_pct']:.2f}% exceeds 1% threshold"
        )

    # 5. Surface continuity
    tests["surface_continuity"] = test_surface_continuity(tire, Fz)

    passed = all(tests.values())
    return ValidationResult(
        passed=passed,
        tests=tests,
        diagnostics=diagnostics,
        warnings=warnings
    )


if __name__ == "__main__":
    result = run_all_combined_tests()
    print("=== Phase 3.4.2 Combined-Slip Validation ===\n")
    for name, ok in result.tests.items():
        print(f"{name:30} : {'PASS' if ok else 'FAIL'}")

    print("\nDiagnostics:")
    for k, v in result.diagnostics.items():
        print(f"  {k}: {v}")

    if result.warnings:
        print("\nWarnings:")
        for w in result.warnings:
            print(f"  ⚠ {w}")

    print("\nOverall:", "ALL PASSED" if result.passed else "SOME FAILED")
