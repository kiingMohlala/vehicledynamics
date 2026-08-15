# PHASE 15.6 — ESC Controller Characterization

**Status: PASS (12/12 gates)**  
**Date:** 2026-08-15  
**Plant:** FROZEN · **K_us:** FROZEN · **K_Mz:** NOT FROZEN  

---

## K_Mz sweep (disturbance −3000 N·m, vx=25)

| K_Mz | e₀ → e_final | max\|Mz\| | sat | flips |
|------|--------------|----------|-----|-------|
| 1000 | 0.210 → 0.255 | 258 | 0 | 0 |
| 2000 | 0.210 → 0.212 | 442 | 0 | 0 |
| **4000** | 0.210 → **0.169** | 726 | 0 | 0 |
| 8000 | 0.210 → 0.118 | 1048 | 0 | 0 |
| 12000 | 0.210 → **0.090** | 1515 | 0 | 0 |

Higher K reduces residual error without oscillation in this envelope.

**Recommended candidate:** K_Mz = 12000 (lowest e_final, sat=0) — **still NOT FROZEN**.

Baseline remains K_Mz = 4000 until an explicit freeze decision.

---

## Other envelopes

- Split-μ: finite, bounded  
- Disturbance magnitude 1.5–7 kN·m: bounded  
- Speed 15/25/35 m/s: bounded  
- Opposite-disturbance recovery asymmetric (documented limitation)  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 15.6 — PASS**

```
tag: v1.5.6-esc-controller-characterization
report: docs/PHASE_15_6.md
artifacts: artifacts/phase_15_6/kmz_sweep.json
```

```
PASSIVE PLANT     FROZEN
ESC ARCHITECTURE  VALIDATED
ESC BASELINE      VALIDATED
ESC GAINS         NOT FROZEN  (candidate 12000 reported)
```
