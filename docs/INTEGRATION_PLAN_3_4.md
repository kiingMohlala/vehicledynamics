# Phase 3.4 Integration Plan – Combined-Slip Dugoff

**Status:** Draft – waiting for visual validation sign-off

This document defines exactly how the combined-slip tire model will be integrated into `BrakeSimulation` once the visual checklist is completed.

---

## 1. Files That Will Change

| File | Change Type | Description |
|------|-------------|-------------|
| `braking/simulation.py` | Modify | Replace pure-longitudinal tire call with combined-slip interface |
| `braking/result.py` | Optional | Add optional fields for Fy / utilization if useful for logging |
| `braking/validation.py` | Extend | Add regression check that α = 0 recovers previous longitudinal results |
| `tire/factory.py` | No change | Already returns the combined-slip capable model |
| `tire/dugoff.py` | No change | Already implements the combined-slip equations |

No other files should be modified during the initial integration.

---

## 2. How the Tire Interface Will Be Called

Current (simplified) call site in `BrakeSimulation.run()`:

```python
state_f = self.tire.longitudinal_lateral_force(slip_f, 0.0, Fz_f)
state_r = self.tire.longitudinal_lateral_force(slip_r, 0.0, Fz_r)
```

For pure braking the slip angle remains zero:

```python
alpha_f = 0.0
alpha_r = 0.0

state_f = self.tire.longitudinal_lateral_force(slip_f, alpha_f, Fz_f)
state_r = self.tire.longitudinal_lateral_force(slip_r, alpha_r, Fz_r)
```

The interface already accepts a non-zero slip angle, so later steering-while-braking scenarios will only require computing α from vehicle states.

---

## 3. Slip Angle Computation (Current Scope)

For the initial integration:

- α is hard-coded to `0.0` (pure longitudinal braking).
- The full combined-slip capability is present but not exercised.

Future extension (bicycle model / ESC):

```python
alpha = atan2(Vy, max(|Vx|, v_eps))
```

This will be introduced only after the pure-braking regression is confirmed.

---

## 4. Regression Tests That Must Be Re-run

After integration, the following must still pass:

### Phase 3.0 Braking
- Static axle loads
- Weight transfer
- Tire force saturation / friction limit
- Energy conservation / passivity
- Emergency stop distance (within previously accepted tolerance)

### Phase 3.2 ABS
- Finite stopping distance
- No prolonged wheel lock
- Pressure modulation present
- Repeatability

### Phase 3.3 Compatibility
- With α = 0, Fx must match the previous longitudinal-only results within the established tolerance (RMS / max error).

### New Combined-Slip Smoke Test
- Simulation still completes with α = 0.
- No NaNs or infinite values appear in any logged quantity.

---

## 5. Definition of Successful Integration

Integration is considered successful only when all of the following are true:

1. All existing Phase 3.0 / 3.2 / 3.3 regression tests still pass.
2. The α = 0 compatibility check against the frozen Phase 3.3 baseline is within tolerance.
3. No new numerical warnings or clamp activations appear under the standard 80 km/h dry-asphalt scenario.
4. The public interfaces (`TireModel`, `BrakeSimulation`, `BrakeSimulationResult`) remain unchanged for existing callers.

---

## 6. Explicit Non-Goals for This Integration Step

- Do **not** introduce lateral vehicle dynamics yet.
- Do **not** change ABS thresholds or gains.
- Do **not** retune tire parameters.
- Do **not** enable non-zero slip angles in the default braking scenarios.

Those belong to later phases (bicycle model, ESC, steering-while-braking).

---

## 7. Execution Sequence (once visual review is signed off)

1. Apply the minimal changes listed in Section 1.
2. Run the full Phase 3 regression suite.
3. Record results against the frozen baseline in `baseline/phase3/`.
4. If any regression appears, stop and diagnose before proceeding.
5. Only after a clean regression pass, freeze as:

   **Phase 3.4 – Combined-Slip Integrated**

---

## Sign-off Gate

This plan may be executed only after:

- [ ] Numerical validation of combined-slip model has passed
- [ ] Visual checklist in `baseline/phase3/plots/VISUAL_CHECKLIST.md` has been completed with no red flags

Date plan approved: __________  
Approved by: __________
