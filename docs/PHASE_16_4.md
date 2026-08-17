# PHASE 16.4 — Broad Vehicle-Level Regression & Scenario Campaign

**Status: PASS (17/17 gates)**  
**Date:** 2026-08-17  
**K_Mz = 10000 (NOT FROZEN)** · **14.9 FROZEN**

---

## Campaign

39 runs (ESC OFF/ON pairs across S01–S20) · deterministic rerun match.

| Metric (ESC ON) | Value |
|-----------------|-------|
| Worst peak \|e_r\| | 1.37 |
| Worst β | 1.13 |
| Max \|Mz\| | 2231 |
| Max cmd | 0.531 |
| Mz flips | 0 |

---

## Gates

| Gate | Result |
|------|--------|
| Passive identity 3.13 / 8.34 s | PASS |
| Determinism (full campaign ×2) | PASS |
| No NaN · cmd ≤ 1 · Mz bounded | PASS |
| Oscillation envelope | PASS (0 flips) |
| Authority inhibit | PASS |
| ABS coexistence | PASS |
| Minimal intervention (straight / brake) | PASS |
| ESC unavailable safe | PASS |
| Cross-scenario yaw/β bounds | PASS |
| ESC OFF zero Mz | PASS |

---

## Verdict

**PHASE 16.4 — PASS**

```
K_Mz candidate:     10000
K_Mz frozen:        NO
14.9 plant:         FROZEN
ESC-OFF regression: 3.13 / 8.34 s
tag:                v1.6.4-esc-regression-campaign
```

Ready for **16.5** formal freeze decision (no further gain hunting).
