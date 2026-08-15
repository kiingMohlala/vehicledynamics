# PHASE 15.7 — ESC Gain Robustness & Candidate Selection

**Status: PASS (10/10 gates)**  
**Date:** 2026-08-15  
**Plant:** FROZEN · **Architecture:** unchanged · **K_Mz:** NOT FROZEN  

---

## Matrix

- K_Mz ∈ {4000, 6000, 8000, 10000, 12000}  
- |ΔMz_dist| ∈ {1.5, 3, 5, 7} kN·m · both signs  
- vx ∈ {15, 25, 35} m/s  
- Steering sweep · split-μ  
- **131** characterization cells  

---

## Robustness scores (nominal −dist, all speeds/M)

| K_Mz | mean e_final | max e_final | sat | flips |
|------|--------------|-------------|-----|-------|
| 4000 | 0.147 | 0.180 | 0 | 0 |
| 6000 | 0.135 | 0.153 | 0 | 0 |
| 8000 | 0.119 | 0.132 | 0 | 0 |
| **10000** | **0.108** | 0.117 | 0 | 0 |
| **12000** | **0.104** | 0.117 | 0 | 0 |

Lowest gain within 15% of best mean residual: **10000**.  
Best mean residual: **12000**.

---

## Gates

| Gate | Result |
|------|--------|
| No NaN / instability | PASS |
| No sustained Mz oscillation | PASS (0 flips) |
| Brake cmd ≤ 1 | PASS |
| Recovery ≰ free response | PASS |
| Split-μ bounded | PASS |
| Inhibits intact | PASS |
| Determinism | PASS |
| Regression | **3.13 / 8.34 s** |

---

## Decision

```
Nominated range:     K_Mz = 10000 → 12000
Best single mean:    K_Mz = 12000
Lowest robust:       K_Mz = 10000
K_Mz frozen:         NO
14.9 plant:          FROZEN
15.5 safety:         PRESERVED
```

Freeze only after a dedicated freeze decision with broader evidence.

---

## Artifacts

- `artifacts/phase_15_7/gain_robustness.csv`
- `artifacts/phase_15_7/gain_robustness.json`
- `artifacts/phase_15_7/nomination.json`

```
tag: v1.5.7-esc-gain-robustness
```
