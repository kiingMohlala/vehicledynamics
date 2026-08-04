# Phase 7.4 Status

## Phase 7.4 – Tire Relaxation Length: Implementation Validated ✅

**Frozen:** 2026-08-04

Optional first-order lag on κ and α before any steady-state tire model.
Dugoff and Pacejka equations are untouched.

### Physics

```
κ̇_eff = (Vx / Lx) · (κ − κ_eff)
α̇_eff = (Vx / Ly) · (α − α_eff)
```

Integrated with the exponential map per timestep (stable for large dt/τ).

### Integration path

```
Measured κ, α  →  Relaxation ODE  →  κ_eff, α_eff  →  Dugoff / Pacejka  →  Fx, Fy
```

### API

```python
from vehicle_dynamics.tire import TransientTire, DugoffTire, RelaxationParams

tire = TransientTire(DugoffTire(), RelaxationParams(Lx=0.3, Ly=0.5))
state = tire.update(kappa, alpha, Fz, vx=20.0, dt=0.01)
# tire.longitudinal_lateral_force(...) still returns pure steady-state (no lag)
```

`enabled=False` or `L → 0` reproduces the baseline exactly.

### Modules

```
tire/relaxation.py
tire/relaxation_state.py
tire/relaxation_parameters.py
tire/transient_tire.py
tire/validation_relaxation.py
```

### Validation (10/10 PASS)

| Gate | Result |
|------|--------|
| disabled = baseline | PASS |
| zero L → baseline | PASS |
| steering step lag | PASS |
| braking step lag | PASS |
| steady-state convergence | PASS |
| higher speed → faster | PASS |
| left/right symmetry | PASS |
| numerical robustness | PASS |
| no NaN/Inf | PASS |
| API steady path | PASS |

### Tag

```bash
git tag -a v0.7.4-phase7.4-relaxation-length \
  -m "Phase 7.4 Tire Relaxation Length: Implementation Validated"
git push origin v0.7.4-phase7.4-relaxation-length
```

### Tire subsystem complete

| Capability | Status |
|------------|--------|
| Dugoff steady-state | ✅ |
| Pacejka MF steady-state | ✅ |
| Camber thrust | ✅ |
| Relaxation length | ✅ |

Next recommended: **Phase 8.0 – Flexible chassis (beam FEM)** or aerodynamics.
