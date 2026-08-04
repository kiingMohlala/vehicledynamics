# Phase 7.6 Status

## Phase 7.6 – Full Combined-Slip Pacejka: Implementation Validated ✅

**Frozen:** 2026-08-04

### Physics

```
Fx = Gx(α) · Fx_pure
Fy = Gy(κ) · Fy_pure

Gx = 1 / sqrt(1 + (α/αc)²)
Gy = 1 / sqrt(1 + (κ/κc)²)
```

Defaults: `αc = 0.15 rad`, `κc = 0.12`

Safety friction clamp retained as numerical net only.

### Regression

```
combined_slip=False  →  Phase 7.5 (pure MF + clamp)
```

Pure braking (α=0) and pure cornering (κ=0) match pure MF exactly.

### Diagnostics

`TireState`: `combined_Gx`, `combined_Gy`, `Fx_pure`, `Fy_pure`

### Validation (11/11 PASS)

| Gate | Result |
|------|--------|
| zero combined = pure | PASS |
| pure braking unchanged | PASS |
| pure cornering unchanged | PASS |
| trail braking reduces Fx | PASS |
| trail braking reduces Fy | PASS |
| weighting monotonic | PASS |
| clamp rate reduced vs pure | PASS (16.6% vs 81.7%) |
| utilization ≤ 1 | PASS |
| disabled = Phase 7.5 | PASS |
| no NaN/Inf | PASS |
| regression smoke | PASS |

### Tag

```bash
git tag -a v0.7.6-phase7.6-combined-slip-pacejka \
  -m "Phase 7.6 Full Combined-Slip Pacejka: Implementation Validated"
git push origin v0.7.6-phase7.6-combined-slip-pacejka
```
