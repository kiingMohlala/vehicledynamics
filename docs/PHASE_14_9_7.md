# PHASE 14.9.7 — Roll-Stiffness Distribution & Lateral Load-Transfer Authority

**Status: PASS (22/22 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.6 PASS · 14.8 frozen  
**Passive only — no ESC, no retuning**

---

## Mission

```
ay → roll moment → front/rear ARB distribution
  → φ + ΔFz_FL/FR/RL/RR → Dugoff → Fy split → ΣMz → yaw
```

Prove **where** load transfer goes, not only that stiffer ARB reduces φ.

---

## Diagnostics added

`dFz_front`, `dFz_rear`, `Fz_front_sum`, `Fz_rear_sum`,  
`Fy_front`, `Fy_rear`, `Mz_tire`, `roll_k_front`, `roll_k_rear`

---

## Key results

| Config | φ | dFz_f | dFz_r | ARB \|F\|_f | ARB \|F\|_r |
|--------|---|-------|-------|-----------|-----------|
| Zero ARB | 0.036 | 2020 | 2289 | — | — |
| Front-heavy | 0.008 | 464 | — | **2391** | 98 |
| Rear-heavy | 0.009 | — | 540 | 102 | **2410** |
| Mech ≈ Hyd @40k | 0.012 | — | — | equal φ | |

- ΣFz conserved  
- L/R symmetry  
- μ / Dugoff coupling  
- Regression **3.13 / 8.34 s**

**Note:** On a rigid sprung body, geometric ΔFz falls as φ falls; ARB force share is the clean distribution metric.

---

## Verdict

**PHASE 14.9.7 — PASS**

```
tag: v1.4.9.7-roll-stiffness-distribution
report: docs/PHASE_14_9_7.md
```

Next logical: **14.9.8 Understeer/Oversteer & Yaw-Stability Characterization**.
