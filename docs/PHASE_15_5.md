# PHASE 15.5 — ESC Split-μ & Failure-Mode Safety Envelope

**Status: PASS (14/14 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 15.4 closed · 14.9 frozen · `K_Mz` **not** retuned  

---

## Question answered

> Does ESC remain safe when symmetric tire authority disappears?

---

## Evidence

| Scenario | Result |
|----------|--------|
| L/R μ asymmetry | stable, \|r\| bounded |
| F/R μ asymmetry | stable |
| One-wheel saturation | util→0.99, no NaN |
| ESC brake saturation | cmd ≤ 1 |
| ABS + ESC | coexistence |
| Inhibit / recovery | util_limit → recover |
| Split-μ mirror | ay +7.05 / −7.05 |
| ESC-OFF under split-μ | ΔΣFy = 0 |
| Regression | **3.13 / 8.34 s** |

---

## Policy unchanged

```
PASSIVE PLANT: FROZEN
ESC ARCHITECTURE: VALIDATED
ESC GAINS: NOT FROZEN
```

---

## Verdict

**PHASE 15.5 — PASS**

```
tag: v1.5.5-esc-split-mu-failure
report: docs/PHASE_15_5.md
```
