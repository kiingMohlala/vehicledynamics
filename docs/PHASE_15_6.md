# PHASE 15.6 — ESC Controller Characterization

**Status: PASS (12/12 gates)**  
**Date:** 2026-08-15  
**Prerequisites:** 15.5 PASS · 14.9 frozen  
**Plant modification:** NONE · **Gain freeze:** NONE  

---

## Boundary

```
Frozen 14.9 plant → 15.1 Obs → 15.3 Decision (K_Mz sweep)
                              → 15.2 Allocation → 15.4 closed-loop
                              → performance metrics
```

Architecture and 15.5 safety gates **unchanged**.

---

## K_Mz sweep (Mz_dist = −3000 N·m, vx = 25, steer = 0.06)

| K_Mz | e₀ → e_final | e_reduction | max\|Mz\| | sat | flips |
|------|--------------|-------------|----------|-----|-------|
| 1000 | 0.210 → 0.255 | −0.045 | 258 | 0 | 0 |
| 2000 | 0.210 → 0.212 | −0.002 | 442 | 0 | 0 |
| **4000** (baseline) | 0.210 → **0.169** | +0.041 | 726 | 0 | 0 |
| 8000 | 0.210 → 0.118 | +0.092 | 1048 | 0 | 0 |
| **12000** | 0.210 → **0.090** | +0.120 | 1515 | 0 | 0 |

Higher gain reduces residual error without Mz sign-switching in this envelope.

---

## Additional envelopes (PASS)

- Split-μ vs nominal: finite / bounded  
- Disturbance magnitude 1.5–7 kN·m: bounded  
- Speed 15 / 25 / 35 m/s: bounded  
- Opposite-disturbance recovery asymmetric (documented; both bounded)  
- ESC-OFF regression: **3.13 / 8.34 s**  
- All candidates pass safety bounds (cmd ≤ 1, flips ≤ 8, e_final < 5)

---

## Deliverables

| Artifact | Path |
|----------|------|
| Harness | `vehicle_dynamics/controls/esc_characterization.py` |
| Validation | `vehicle_dynamics/demonstration/validation_15_6.py` |
| Sweep JSON | `artifacts/phase_15_6/kmz_sweep.json` |
| CSV | `artifacts/phase_15_6/characterization.csv` |
| Recommendation | `artifacts/phase_15_6/recommendation.json` |

---

## Verdict

**PHASE 15.6 — PASS**

```
Best-performing candidate:     K_Mz = 12000
Recommended operating range:   4000 → 12000
K_Mz frozen:                   NO
14.9 plant:                    FROZEN
15.5 safety envelope:          PRESERVED
Regression:                    3.13 / 8.34 s
```

```
tag: v1.5.6-esc-controller-characterization
```
