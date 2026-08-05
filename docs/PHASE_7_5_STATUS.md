# Phase 7.5 Status

## Phase 7.5 – Load-Sensitive Tire Model: Implementation Validated ✅

**Frozen:** 2026-08-04

### Physics

```
μ_eff(Fz) = μ₀ · (Fz₀ / Fz)^n
```

Default: `n = 0.08`, `Fz0 = 4000 N`

- Fz = Fz0 → μ = μ₀
- Higher Fz → lower μ
- Lower Fz → higher μ

### Application

**Dugoff:** `μ` replaced by `μ_eff(Fz)` in λ and friction clamp.

**Pacejka:**
```
Dx = μ_eff_x(Fz) · Fz
Dy = μ_eff_y(Fz) · Fz
```

### Regression

```
load_sensitive=False  →  identical to Phase 7.4
```

### Usage

```python
from vehicle_dynamics.tire import DugoffTire, DugoffParams, effective_mu

tire = DugoffTire(DugoffParams(
    load_sensitive=True,
    Fz0=4000.0,
    load_exponent=0.08,
))
```

### Validation (10/10 PASS)

| Gate | Result |
|------|--------|
| disabled = baseline | PASS |
| Fz = Fz0 | PASS |
| higher load → lower μ | PASS |
| lower load → higher μ | PASS |
| force finite | PASS |
| friction limit | PASS |
| Dugoff disabled unchanged | PASS |
| Pacejka disabled unchanged | PASS |
| no NaN/Inf | PASS |
| regression smoke | PASS |

### Tag

```bash
git tag -a v0.7.5-phase7.5-load-sensitive-tire \
  -m "Phase 7.5 Load-Sensitive Tire Model: Implementation Validated"
git push origin v0.7.5-phase7.5-load-sensitive-tire
```
