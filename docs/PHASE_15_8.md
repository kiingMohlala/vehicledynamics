# PHASE 15.8 — ESC Gain Selection & Final Candidate Validation

**Status: PASS (12/12 gates)**  
**Date:** 2026-08-15  

---

## Candidate metrics (nominal −dist)

| K_Mz | mean e_final | worst e_final | mean max\|Mz\| | brake energy |
|------|--------------|---------------|----------------|--------------|
| **10000** | 0.108 | 0.127 | **1067** | **0.454** |
| 11000 | 0.104 | 0.119 | 1153 | 0.468 |
| 12000 | **0.102** | **0.117** | 1246 | 0.475 |

All three eligible (within 10% mean / 15% worst of best).  
**Selected: 10000** — lowest gain with operationally equivalent recovery and lower actuator demand.

---

## Verdict

**PHASE 15.8 — PASS**

```
Selected candidate:     K_Mz = 10000
Validated range:        10000 → 12000
Gain frozen:            NO
Reason:                 Lowest eligible gain; recovery ≈ best with less brake authority
14.9 plant:             FROZEN
15.5 safety envelope:   PRESERVED
ESC-OFF regression:     3.13 / 8.34 s
```

```
tag: v1.5.8-esc-gain-selection
```
