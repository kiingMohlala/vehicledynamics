# PHASE 15.9 — ESC Transient Event & Recovery Validation

**Status: PASS (12/12 gates)**  
**Date:** 2026-08-17  
**Candidate:** K_Mz = 10000 (**NOT FROZEN**)  

---

## Events

| Event | Result |
|-------|--------|
| Step ± disturbance | activation · recovery |
| Impulse | 0 Mz flips |
| Sustained then remove | ΔMz → 0 |
| Cross-zero | 0 chatter flips |
| Near-threshold | hysteresis (7 act, no chatter) |
| High-util | inhibit True |
| Split-μ | final \|e_r\| ≈ 0.10 |

---

## Verdict

**PHASE 15.9 — PASS**

```
Selected candidate:     K_Mz = 10000
Transient recovery:     PASS
Activation/release:     PASS
Overshoot:              PASS
Chatter:                PASS
Split-μ:                PASS
ABS coexistence:        PASS
ESC-OFF regression:     3.13 / 8.34 s
Gain frozen:            NO
14.9 plant:             FROZEN
```

```
tag: v1.5.9-esc-transient-recovery
```
