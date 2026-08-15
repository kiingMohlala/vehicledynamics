# PHASE 15.3 — Stability Envelope & ESC Decision Logic

**Status: PASS (20/20 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 15.2 PASS · 15.1 · 14.9 frozen  

**No plant intervention · No brake commands · No closed loop**

---

## Boundary

```
15.1 Observation → 15.3 Decision → ΔMz *request* (hypothetical)
                                      │
                                      ✕ not applied to plant
                                      ✕ does not call BrakeAllocator
```

---

## Policy

| Condition | Action |
|-----------|--------|
| \|e_r\| < e_enter (0.12) | idle |
| \|e_r\| > e_enter | active |
| stay active until \|e_r\| < e_exit (0.06) | hysteresis |
| e_r > 0 | request **−ΔMz** |
| e_r < 0 | request **+ΔMz** |
| vx < 8 m/s, low steer, util≥0.98, \|β\|≥0.45 | inhibit |

`ΔMz_req = clip(−K_Mz · e_r, ±max_Mz)` with util soft-taper.

---

## Evidence

- Zero error → zero request  
- Sign correct & symmetric  
- Hysteresis hold between e_exit and e_enter  
- Transient detection on real telemetry (observe-only)  
- ΔΣFy = 0 with decision running  
- Allocator untouched  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 15.3 — PASS**

```
tag: v1.5.3-esc-decision-logic
report: docs/PHASE_15_3.md
```

**Next (15.4):** Closed-loop ESC — connect Decision → Allocator → plant with explicit enable, stability envelope tests, and freeze discipline.
