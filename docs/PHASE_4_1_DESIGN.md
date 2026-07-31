# Phase 4.1 – Load Transfer Coupling

**Status:** Design (not yet implemented)

This document freezes the conventions, equations, and validation plan for lateral load transfer before any code is written.

---

## 1. Objectives

- Add quasi-static lateral load transfer to the validated Phase 4.0 bicycle model.
- Produce dynamic normal loads Fz_f / Fz_r (and optionally left/right if a dual-track extension is introduced later).
- Keep the tire interface unchanged.
- Preserve Phase 4.0 behaviour when lateral acceleration is zero (regression).
- Do **not** introduce a full roll degree of freedom yet (that remains optional / later).

---

## 2. Scope

### In scope
- Quasi-static lateral load transfer based on lateral acceleration at the CG.
- Optional simple roll-moment / roll-angle approximation (algebraic, not dynamic).
- Updated normal loads fed into the combined-slip tire model.
- Validation: zero-ay regression, left/right symmetry, steady-state circular with load transfer, sensible understeer trend.

### Explicitly out of scope
- Dynamic roll DOF (roll rate, roll inertia)
- Suspension roll stiffness / roll damping as dynamic states
- Longitudinal load transfer coupling into the lateral model (belongs with combined braking + steering)
- Aerodynamic downforce
- ESC / torque vectoring

---

## 3. Coordinate & Sign Conventions (unchanged from Phase 4.0)

- +x forward, +y left, +z up
- Positive ay (to the left) increases load on the **right** tires and unloads the **left** tires
- For the single-track bicycle model we only need **axle** normal loads (front total, rear total). Left/right split is optional and can be added later for a dual-track model.

---

## 4. Quasi-Static Lateral Load Transfer

### 4.1 Axle normal loads (minimum viable model)

Total weight:
```
W = m * g
```

Static distribution (already used in Phase 4.0):
```
Fz_f_static = W * (b / L)
Fz_r_static = W * (a / L)
```

Lateral load transfer is primarily a **left/right** effect. For a single-track bicycle model the **axle totals** remain approximately the static values unless we also model longitudinal transfer or aerodynamic effects.

Therefore Phase 4.1 has two levels:

**Level A – Axle totals unchanged, prepare interface**
- Keep Fz_f, Fz_r as static.
- Add the load-transfer *calculation* and diagnostics so dual-track / later phases can consume them.

**Level B – Effective axle load sensitivity (recommended for bicycle)**
- Introduce a simple roll-moment approximation that can slightly modulate front/rear effective normal load if a non-zero roll-axis height is defined, **or**
- Keep axle totals static and document that true left/right transfer requires a dual-track model.

**Recommendation for Phase 4.1:**
Implement Level A cleanly (compute and expose left/right transfer magnitudes) while keeping the bicycle model on static axle totals, **and** provide an optional dual-track normal-load helper for future use. This avoids pretending the single-track model can represent left/right transfer without additional states.

---

## 5. Left/Right Transfer Formulas (for diagnostics & future dual-track)

Total lateral transfer:
```
ΔF_y = (m * ay * h) / t
```
where
- h = CG height [m]
- t = track width [m]
- ay = lateral acceleration at CG [m/s²]

Front/rear share of the transfer (approximate, using roll stiffness distribution or geometric share):
```
ΔF_f = ΔF_y * χ_f
ΔF_r = ΔF_y * (1 - χ_f)
```
with χ_f ∈ [0, 1] = front roll-stiffness ratio (default 0.5).

Then:
```
Fz_fl = Fz_f_static/2 - ΔF_f
Fz_fr = Fz_f_static/2 + ΔF_f
Fz_rl = Fz_r_static/2 - ΔF_r
Fz_rr = Fz_r_static/2 + ΔF_r
```
(with positive ay unloading the left side).

Clamp all Fz ≥ Fz_min (e.g. 50 N) to avoid zero/negative loads.

---

## 6. New Parameters

```python
@dataclass
class LoadTransferParameters:
    h_cg: float = 0.55      # CG height [m]
    track_f: float = 1.55   # front track [m]
    track_r: float = 1.55   # rear track [m]
    chi_f: float = 0.55     # front roll-stiffness ratio [-]
    Fz_min: float = 50.0    # minimum normal load [N]
```

These extend `BicycleParameters` or live in a dedicated dataclass composed into the model.

---

## 7. Interface Impact

### Tire API
**Unchanged.** Still:
```python
longitudinal_lateral_force(slip_ratio, slip_angle, normal_load) -> TireState
```

### Bicycle model
- Compute ay from the current state (or from the previous step for quasi-static lag).
- Compute left/right Fz (diagnostics).
- For Phase 4.1 single-track: continue to pass axle-total Fz to the tire model unless an explicit dual-track mode is enabled.
- Log transfer magnitudes and optional left/right loads in the result object.

### Result object additions
```python
ay: float          # already present as ay_force / ay_vehicle
Delta_F_f: float   # front lateral transfer magnitude
Delta_F_r: float   # rear lateral transfer magnitude
Fz_f: float        # axle total used by tire model
Fz_r: float
# optional:
Fz_fl, Fz_fr, Fz_rl, Fz_rr
```

---

## 8. Validation Plan

1. **Zero-ay regression**  
   With δ = 0 (straight line), load transfer = 0 and Phase 4.0 results are reproduced within tolerance.

2. **Left/right symmetry**  
   +δ vs −δ produces opposite transfer signs and mirrored forces.

3. **Steady circular with transfer diagnostics**  
   Constant steer → constant ay → constant ΔF. No divergence.

4. **Fz bounds**  
   No normal load falls below Fz_min under the validated maneuver set.

5. **Parameter sensitivity**  
   Increasing h_cg increases |ΔF|; increasing track decreases |ΔF|.

---

## 9. Success Criteria

Phase 4.1 is complete when:

- Load-transfer formulas are implemented and unit-tested.
- Zero-ay regression against Phase 4.0 passes.
- Symmetry and circular diagnostics pass.
- Tire API remains unchanged.
- No new dynamic DOFs have been introduced.

---

## 10. Open Points to Confirm Before Coding

1. **Level A vs Level B** – Keep bicycle on static axle totals and only expose left/right diagnostics (recommended), or attempt an effective axle-load modulation?
2. **Default h_cg, track, χ_f** values.
3. **Whether ay used for transfer is ay_force or ay_vehicle** (recommend ay_force for consistency with tire forces).

Once confirmed, implementation can begin under the same validation-first workflow.
