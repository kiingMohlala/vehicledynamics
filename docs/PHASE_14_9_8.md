# PHASE 14.9.8 — Understeer/Oversteer & Yaw-Stability Characterization

**Status: PASS (17/17 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.7 PASS · 14.8 frozen  
**Passive only — no ESC, no retuning**

---

## Mission

Characterize the frozen plant's natural handling balance:

```
δ, vx → α, Fz → Dugoff → Fy front/rear → Mz → understeer / neutral / oversteer
```

Classification via steering gradient:

```
dδ/d(ay) > 0  → understeer
dδ/d(ay) ≈ 0  → neutral
dδ/d(ay) < 0  → oversteer
```

---

## Result (authoritative hypercar)

| Metric | Value |
|--------|-------|
| Classification @ 25 m/s | **understeer** |
| dδ/d(ay) | **+0.0065 rad/(m/s²)** |
| Max \|ay\| (μ=1.15) | 11.6 m/s² |
| Max \|ay\| (μ=0.55) | 5.7 m/s² |
| Max util front/rear | ~0.97 |
| Yaw stability | max \|r\| = 0.56 rad/s |
| Recovery (steer off) | ay 11.6 → 0.10 |
| Regression | **3.13 / 8.34 s** |

---

## Module

`vehicle_dynamics/lateral/handling_characterization.py`

- constant-speed steer sweeps  
- steering gradient fit  
- yaw-gain vs speed  

---

## Verdict

**PHASE 14.9.8 — PASS**

Natural handling is **understeer** in the linear/moderate region; tire-limit utilization is reached at higher δ without yaw runaway.

```
tag: v1.4.9.8-understeer-oversteer-yaw-stability
report: docs/PHASE_14_9_8.md
```

Next (optional): 14.9.9 combined braking + cornering before ESC.
