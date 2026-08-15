# PHASE 14.5 — Transient Vehicle Body Dynamics & Suspension Load Transfer

**Status: PASS (18/18 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.4 PASS  
**No retuning** of frozen 14.2 mass / power / μ / tire / gear parameters.

---

## Mission

Wheel loads now **emerge** from sprung-mass heave/pitch/roll dynamics:

```
ax, ay, aero
      ↓
sprung-mass (z, θ, φ)
      ↓
suspension forces (k, c, roll stiffness)
      ↓
Fz_FL / FR / RL / RR
      ↓
Dugoff → Fx / Fy
```

14.4 algebraic quasi-static transfer remains available as fallback (`use_sprung_body=False`).

---

## Implementation

**`vehicle_dynamics/simulation/sprung_body.py`**

| State | Meaning |
|-------|---------|
| z, ż | heave from static ride |
| θ, θ̇ | pitch (nose-up +) |
| φ, φ̇ | roll (right-down +) |

Corner deflection: `δ = z ∓ a·θ ± (t/2)·φ`  
Force on body: `F = −kδ − cδ̇` (+ anti-roll couple)  
`Iθ·θ̈ = M_susp + m·ax·h_cg`  
`Iφ·φ̈ = M_susp + m·ay·h_cg`

**Authority:** `SuspensionConfig` → `SimulationConfig` → `DualTrackConfig` → `SprungBodyModel`

Hypercar rates (sports-car settle near quasi-static steady state):

- k_front/rear = 80 / 95 kN/m  
- c_front/rear = 6 / 7 kN·s/m  
- roll stiffness F/R = 35 / 30 kN·m/rad  

---

## Gates (18/18)

| Gate | Result |
|------|--------|
| Architecture | sprung body in DualTrackPlant |
| Suspension parameter authority | k, c, h_cg bound |
| Heave / pitch / roll dynamics | all active |
| Pitch symmetry | accel θ>0, brake θ<0 (nose dive) |
| Roll symmetry | φ(+steer) ≈ −φ(−steer) |
| Dynamic wheel-load coupling | L/R Fz split under corner |
| Conservation | ΣFz ≈ mg at rest |
| Tire Fz coupling | stiffer k → less pitch |
| Transient braking | front load ↑, θ < 0 |
| Transient cornering | φ + ΔFz |
| Combined brake+steer | min_Fz ≥ 50, no NaN |
| Dissipation | E_damp ≥ 0 |
| Mutation / negative fallback | PASS |
| Frozen regression | hist 5.37 s; hyper 3.24 s |
| Deterministic ×5 | identical |

---

## Regression note

Transient pitch slightly changes longitudinal traction during launch/shifts vs pure quasi-static 14.4:

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | **3.24 s** (was 3.13) | **8.47 s** (was 8.30) |
| Historical | **5.37 s** | **18.86 s** |

Within documented tolerance. **Not retuned** to recover prior numbers — this is physical consequence of body dynamics.

---

## Limitations

1. No unsprung wheel-hop DOFs (explicit non-goal).  
2. Small-angle kinematics; no full suspension geometry.  
3. Linear springs/dampers.  
4. Global energy ledger still PARTIAL (14.2H.2).

---

## Verdict

**PHASE 14.5 — PASS**

```
tag: v1.4.5-transient-body-dynamics
report: docs/PHASE_14_5.md
```
