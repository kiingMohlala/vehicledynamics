# PHASE 14.8 — Full Coupled Plant Integrity & Authority Audit

**Status: PASS (34/34 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.7 PASS  
**Scope:** System-wide validation only. **No new physics. No retuning.**

---

## Mission

Prove a single authoritative runtime plant:

```
VehicleDefinition → SimulationConfig → Runtime Plant
  Aero (β) ↔ Sprung body ↔ Suspension ↔ Unsprung ↔ Tire Fz → Dugoff
  Powertrain → wheel torque → κ → Fx
```

---

## Results

### Authority (26/26 parameters)

mass, μ, Cx/Cy, radius, wheelbase, tracks, h_cg, drive split, brake Tmax,  
FD, gear ratios, k/c suspension, m_u, k_tire/c_tire, Cd, Cyβ, Cnβ,  
peak power, ABS, use_sprung, use_unsprung — **config ≡ runtime**.

### Poisoned defaults

mass=9999, μ=0.01, k_tire=1, m_u=1, h_cg=9 → **ignored**; hypercar retains authoritative values.

### Coupling chain

| Link | Evidence |
|------|----------|
| Aero → body | Fy=315 N, ay=8.65 |
| Body → suspension | brake θ=−0.020 |
| Suspension → unsprung | zu_FL=0.029 |
| Unsprung → tire | Fz_FL=5190 under bump |
| Tire → Dugoff | ax=10.98 under drive |
| Crosswind → β → Mz → ay | β=−0.41, Mz=191, ay=10.4 |

### No competing Fz

Dynamic Fz under bump ≠ quasi-static; poisoned static Fz still responds to road.

### Mutations

| Change | Effect |
|--------|--------|
| Cyβ ×2 | Fy 341 → 655 |
| h_cg ×2 | \|θ\| 0.013 → 0.027 |
| k_tire ↑ | hop T 0.126 → 0.052 s |

### Energy boundary (PARTIAL global)

E_spring, E_damper_susp, E_tire_spring, E_tire_damper, E_body_K, E_unsprung_K all ≥ 0.  
**Global drivetrain energy: PARTIAL** (unchanged from 14.2H.2).

### Regression (14.7 baseline)

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | **3.13 s** | **8.34 s** |
| Historical | **5.37 s** | **19.81 s** |

---

## Core plant freeze

```
14.2  Runtime authority
14.3  Relative-airflow aero
14.4  Load transfer
14.5  Sprung-body dynamics
14.6  Body-model integrity
14.7  Unsprung / wheel-hop
14.8  Full coupled-plant audit  ← FROZEN FOUNDATION
```

---

## Known limitations (accepted)

1. Global drivetrain energy PARTIAL  
2. Linear tire vertical / suspension  
3. Open-loop manoeuvres  
4. No ESC / active aero / wheel-hop nonlinear contact  

---

## Verdict

**PHASE 14.8 — PASS**

**Core vehicle plant: FROZEN**

```
tag: v1.4.8-coupled-plant-validated
report: docs/PHASE_14_8.md
```
