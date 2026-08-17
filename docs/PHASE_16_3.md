# PHASE 16.3 — ESC Failure Injection & Degraded-Authority Validation

**Status: PASS (16/16 gates)**  
**Date:** 2026-08-17  
**K_Mz = 10000 (NOT FROZEN)** · **14.9 FROZEN**

---

## Principle

```
normal → degraded → unavailable
  ESC      limited      INHIBIT
              ↓
           recover → re-entry
```

ESC must not demand authority the tires cannot provide.

---

## Results

| Failure class | Result |
|---------------|--------|
| ESC unavailable | cmd = 0 |
| Stale obs / hysteresis release | activate → release |
| Single-wheel / axle / severe split-μ | finite · bounded |
| Reduced actuator / saturation | cmd ≤ 1 |
| util limit vs large e_r | **INHIBIT** |
| Inhibit → no stuck-active | PASS |
| Post-fault no unexpected Mz | PASS |
| ABS + ESC | min_Fz = 1615 |
| Split-μ + actuator limit | peak \|r\| ≈ 0.27 |
| Regression | **3.13 / 8.34 s** |

---

## Verdict

**PHASE 16.3 — PASS**

```
K_Mz candidate:     10000
K_Mz frozen:        NO
14.9 plant:         FROZEN
tag:                v1.6.3-esc-failure-injection
```
