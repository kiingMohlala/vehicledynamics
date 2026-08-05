# Phase 7.7 Status

## Phase 7.7 – Tire Aligning Torque (Mz): Implementation Validated ✅

**Frozen:** 2026-08-04

### Physics

```
Mz = −Fy · tp
tp = t0 · exp(−(|α|/αt)²)
```

Defaults: `t0 = 0.05 m`, `αt = 0.15 rad`

### Properties

| Condition | Behavior |
|-----------|----------|
| α = 0 | trail = t0, Mz = 0 (Fy = 0) |
| moderate α | finite restoring Mz |
| large α | trail → 0, Mz → 0 |
| sign(Fy) flips | Mz flips |
| combined slip | reduced Fy → reduced Mz |

### Regression

```
aligning_torque=False  →  Phase 7.6 forces, Mz = 0
```

Dugoff unchanged.

### TireState additions

- `Mz`
- `pneumatic_trail`

### Validation (9/9 PASS)

| Gate | Result |
|------|--------|
| disabled = Phase 7.6 | PASS |
| zero slip → Mz ≈ 0 | PASS |
| moderate α → finite Mz | PASS |
| sign reversal | PASS |
| trail decreases with α | PASS |
| large α → Mz reduces | PASS |
| combined-slip compatibility | PASS |
| no NaN/Inf | PASS |
| regression smoke | PASS |

### Tag

```bash
git tag -a v0.7.7-phase7.7-aligning-torque \
  -m "Phase 7.7 Tire Aligning Torque (Mz): Implementation Validated"
git push origin v0.7.7-phase7.7-aligning-torque
```

### Tire subsystem complete

| Capability | Status |
|------------|--------|
| Dugoff | ✅ |
| Pacejka Magic Formula | ✅ |
| Camber thrust | ✅ |
| Relaxation length | ✅ |
| Load sensitivity | ✅ |
| Combined-slip Pacejka | ✅ |
| Self-aligning torque (Mz) | ✅ |
