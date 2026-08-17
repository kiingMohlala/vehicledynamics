# PHASE 16.2 — Advanced Limit-Handling & Combined-Maneuver Validation

**Status: PASS (15/15 gates)**  
**Date:** 2026-08-17  
**K_Mz = 10000 (NOT FROZEN)** · **14.9 plant FROZEN** · no architecture changes  

---

## Scenarios

| Scenario | peak \|r\| | peak util | max \|Mz\| | flips |
|----------|-----------|-----------|-----------|-------|
| Progressive saturation | bounded | 0.98 | 1897 | 0 |
| High-speed transient | bounded | — | — | 0 |
| Brake+steer near sat | bounded | — | — | 0 · ABS min_Fz=1608 |
| μ high→low / low→high | stable | — | — | 0 |
| Split-μ transitions | 0.32 | — | — | 0 |
| Steering reversal | 0.49 | — | — | 0 |
| Inhibit → recovery | inhibit ✓ · reentry ✓ | — | — | — |

---

## Verdict

**PHASE 16.2 — PASS**

```
K_Mz candidate:     10000
K_Mz frozen:        NO
14.9 plant:         FROZEN
ESC-OFF regression: 3.13 / 8.34 s
```

ESC remains useful near limits, inhibits under high util, recovers when authority returns, and does not chatter under steering reversal or μ transitions.

```
tag: v1.6.2-limit-handling
```
