# PHASE 15.2 — ESC Differential-Brake Command Authority

**Status: PASS (18/18 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 15.1 PASS · 14.9 frozen  
**Command path only — no feedback on e_r**

---

## Boundary

```
15.1 observation (read-only)
        │
        ▼
15.2 ESCCommand(ΔMz)  ← external / future decision layer
        │
   BrakeAllocator     ← NO e_r, β, ay dependency
        │
   brake_cmd[FL,FR,RL,RR]
        │
   DualTrackPlant.brake_add overlay
        │
   frozen plant + ABS
```

**Question answered:** “Can I request this yaw moment?”  
**Not answered:** “Should ESC request this yaw moment?”

---

## Allocation (plant geometry)

| Request | Brakes | Yaw response |
|---------|--------|--------------|
| +ΔMz | FL + RL | r > 0 |
| −ΔMz | FR + RR | r < 0 |
| 0 | none | baseline |

Moment conservation: achieved ≈ requested (unsaturated).

---

## Evidence

- Allocator independent of e_r (source inspection)  
- Zero command = zero intervention = baseline ΣFy  
- Plant yaw response: r(+3000)≈+2.0, r(−3000)≈−2.0  
- ABS path intact  
- Observer remains read-only  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 15.2 — PASS**

```
tag: v1.5.2-esc-command-authority
report: docs/PHASE_15_2.md
```

**Next (15.3):** Stability envelope / decision logic — when to request ΔMz — still open-loop policy tests, not full closed-loop ESC.
