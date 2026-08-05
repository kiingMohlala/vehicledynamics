# Phase 3.4 Status

## Phase 3.4 – Combined-Slip Dugoff

**Status: Integrated & Regression Validated** ✅

| Gate | Status |
|------|--------|
| Numerical validation (Phase 3.4.2) | ✅ Passed (`tire/validation_combined.py`) |
| Visual validation | ✅ Passed |
| Integration into BrakeSimulation | ✅ Complete (α = 0 in pure braking) |
| Full regression suite | ✅ Passed (committed suites only) |

## Freeze Date
2026-07-31 (remediated 2026-08-03)

## Regression Summary (committed, re-runnable)

### Phase 3.0 – `braking/validation.py`
| Test | Result |
|------|--------|
| Static axle loads | PASS |
| Weight transfer conservation | PASS |
| ABS emergency-stop smoke | PASS |

### Phase 3.2 – `braking/validation_abs.py`
| Test | Result |
|------|--------|
| pressure_bounds | PASS |
| pressure_modulation | PASS |
| state_machine | PASS |

### Phase 3.3 – `tire/validation_longitudinal.py`
| Test | Result |
|------|--------|
| zero_slip | PASS |
| friction_limit | PASS |
| symmetry | PASS |
| linear_region | PASS |

### Phase 3.4 – `tire/validation_combined.py`
| Test | Result |
|------|--------|
| Phase 3.3 compatibility (α = 0) | PASS |
| Pure lateral | PASS |
| Combined-slip coupling | PASS |
| Friction limit + clamp stats | PASS |
| Surface continuity | PASS |

**Overall: ALL COMMITTED REGRESSION CHECKS PASSED**

## Documentation integrity note

Prior freeze records listed ABS and Phase 3.3 checks as PASS without corresponding suite files in the repository. Those gaps were closed on 2026-08-03 by committing `validation_abs.py` and `validation_longitudinal.py` and aligning this status document and `baseline/phase3/validation_summary.json` with the code that actually runs.

## Integration Notes

- `BrakeSimulation` consumes the combined-slip API with α = 0 for pure braking.
- No tire physics were changed during integration.
- Previous tire models remain selectable via the factory.
- Public interfaces remain stable.

## Known Limitations (still apply)

- Small-angle approximation for lateral stiffness (Fy0 = Cy·α)
- No relaxation length / transient dynamics
- Generic tire parameters (not fitted to experimental data)

## How to re-run

```bash
python -m vehicle_dynamics.braking.validation
python -m vehicle_dynamics.braking.validation_abs
python -m vehicle_dynamics.tire.validation_longitudinal
python -m vehicle_dynamics.tire.validation_combined
```
