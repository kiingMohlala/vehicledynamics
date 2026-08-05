# Phase 5.6 Status

## Phase 5.6 – Longitudinal Load Transfer: Implementation Validated ✅

**Frozen:** 2026-08-04

### Physics

```
ΔFz = m · ax · h_cg / L
Fz_f = W·b/L − ΔFz
Fz_r = W·a/L + ΔFz
```

Then Phase 4.1 lateral transfer on those axle loads.

- `ax > 0` → acceleration → load to rear
- `ax < 0` → braking → load to front
- `ax = 0` → Phase 5.0 behaviour (regression)

Dual-track uses lagged `ax = ΣFx / m` during integration.

### Modules

```
dual_track/normal_loads.py          # + longitudinal_axle_loads, ax arg
dual_track/validation_longitudinal_load.py
dual_track/simulation.py            # feeds ax
```

### Validation (8/8 PASS)

| Gate | Result |
|------|--------|
| zero_ax_matches_phase50 | PASS |
| braking_loads_front | PASS |
| accel_loads_rear | PASS |
| total_weight_conserved | PASS |
| theoretical_longitudinal | PASS |
| combined_lateral_longitudinal | PASS |
| sign_symmetry_ax | PASS |
| fz_positive | PASS |

### Tag

```bash
git tag -a v0.5.6-phase5.6-longitudinal-load \
  -m "Phase 5.6 Longitudinal Load Transfer: Implementation Validated"
git push origin v0.5.6-phase5.6-longitudinal-load
```

### Next (roadmap)

6.6 Nonlinear suspension geometry → 6.7 Jacking → 7.2 Handling metrics → …
