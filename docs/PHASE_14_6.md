# PHASE 14.6 — Dynamic Body Model Integrity & Energy Closure

**Status: PASS (24/24 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.5 PASS  
**Scope:** Validation only — no new subsystems, no retuning of vehicle identity parameters.

---

## Mission

Prove the 14.5 heave/pitch/roll model is numerically stable, converges to quasi-static Fz, conserves vertical load (including aero), and exchanges energy consistently with the suspension.

---

## Physics corrections applied during validation

1. **Pitch kinematics:** `+θ` (nose-up) raises the front (`δ_front = z + a·θ`).  
2. **Tire load sign:** `Fz = Fz_static + F_sd` so spring compression increases contact load.  
3. **Aero path:** downforce acts on the body equation only; spring compression carries it into `Fz` (no double-count).

After correction, steady `Fz` under constant `ax` matches 14.4 quasi-static to **0.00 N**.

---

## Gates (24/24)

| Category | Gates | Result |
|----------|-------|--------|
| Stability | numerical, static eq, 50 s no drift | PASS |
| Transients | heave, pitch accel/brake, roll, symmetry, damping decay | PASS |
| Authority | suspension params, h_cg, k mutation | PASS |
| Loads | conservation (static/accel/brake/corner/aero/combined), aero balance, combined transfer | PASS |
| Consistency | QS convergence 0 N, QS fallback, timestep | PASS |
| Energy | spring ≥0, damper ≥0, body KE ≥0 (PARTIAL vs full drivetrain) | PASS |
| Replay | deterministic ×5 | PASS |
| Regression | historical + hypercar frozen refs | PASS |

---

## Quasi-static convergence

| Condition | max \|Fz_dyn − Fz_qs\| |
|-----------|----------------------|
| ax = 5 m/s² steady | **0.00 N** |

`use_sprung_body=False` path also matches QS (14.4 fallback intact).

---

## Energy (body/suspension scope)

| Term | Status |
|------|--------|
| E_spring | instrumented ≥ 0 |
| E_damper (dissipated) | ≥ 0, non-generative |
| E_heave / E_pitch / E_roll | ≥ 0 |
| Global drivetrain residual | still **PARTIAL** (14.2H.2) |

---

## Regression (14.5 references)

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | **3.16 s** (ref 3.24) | **8.39 s** (ref 8.47) |
| Historical | **5.37 s** | **19.81 s** |

Within tolerance; Fz sign fix slightly improved longitudinal fidelity vs pure 14.5.

---

## Verdict

**PHASE 14.6 — PASS**

```
VehicleDefinition → SimulationConfig → DualTrackPlant → SprungBodyModel
  → Fz → Dugoff → vehicle response
  → validated energy + deterministic dynamics + QS convergence
```

```
tag: v1.4.6-body-integrity-energy
report: docs/PHASE_14_6.md
```
