# Phase 6.1 Status

## Phase 6.1 – Wheel Rate & Motion Ratio: Implementation Validated ✅

**Frozen:** 2026-08-03

### Definitions

```
IR = z_spring / z_wheel     (installation ratio)
MR = z_wheel / z_spring = 1/IR

Kw = Ks × IR² = Ks / MR²
Cw = Cs × IR² = Cs / MR²
```

Direct-acting: IR = MR = 1 → Kw = Ks, Cw = Cs  
Pushrod IR = 0.7 → Kw = 0.49 × Ks

### Module

`vehicle_dynamics/suspension/wheel_rate.py`

Geometry solver (6.0) unchanged.

### Validation (9/9 PASS)

| Gate | Result |
|------|--------|
| mr_equals_one | PASS |
| mr_less_than_one | PASS |
| mr_greater_than_one | PASS |
| analytical_crosscheck (energy) | PASS |
| damping_same_ratio | PASS |
| left_right_symmetry | PASS |
| monotonicity | PASS |
| mr_inverse_of_ir | PASS |
| zero_ir_raises | PASS |

### Tag

```bash
git tag -a v0.6.1-phase6.1-wheel-rate \
  -m "Phase 6.1 Wheel Rate & Motion Ratio: Implementation Validated"
git push origin v0.6.1-phase6.1-wheel-rate
```

### Next

Phase 6.2 – Geometry coupling to vehicle (camber/toe/MR into dual-track)
