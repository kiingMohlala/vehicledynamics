# PHASE 16.1 — Baseline Handling Scenario Suite

**Status: PASS (15/15 gates)**  
**Date:** 2026-08-17  
**Candidate:** K_Mz = 10000 (**NOT FROZEN**)  
**Plant:** 14.9 FROZEN · Architecture 15.1–15.9 unchanged  

---

## Suite (ESC OFF vs ON)

| ID | Scenario |
|----|----------|
| S1 | Step steer · 15/25/35 m/s · ±δ |
| S2 | Sine steer · mild/aggressive |
| S3 | Lane-change · 25/35 m/s |
| S4 | Steady corner · low/mid/high δ |
| S5 | Straight braking |
| S6 | Brake + steer |
| S7 | Split-μ L/R · mirror · F/R |
| S8 | Disturbance recovery |

**42** paired runs.

---

## Safety

| Gate | Result |
|------|--------|
| ESC OFF ≡ passive · zero Mz | PASS |
| Regression 3.13 / 8.34 s | PASS |
| Determinism | PASS |
| No NaN · cmd ≤ 1 · Mz ≤ max | PASS |
| 0 pathological Mz flips | PASS |
| ABS coexistence | PASS |
| Inhibits intact | PASS |
| Split-μ bounded | PASS |
| Straight-brake minimal intervention | PASS |
| Disturbance correction active | PASS |

---

## Note

Steady corner can keep ESC active because measured `r` vs understeer `r_ref` leaves residual `e_r` — intervention is bounded and non-oscillatory (not zero by design).

---

## Verdict

**PHASE 16.1 — PASS**

```
K_Mz candidate:     10000
K_Mz frozen:        NO
14.9 plant:         FROZEN
15.5 safety:        PRESERVED
ESC-OFF regression: 3.13 / 8.34 s
```

```
tag: v1.6.1-esc-scenario-suite
```
