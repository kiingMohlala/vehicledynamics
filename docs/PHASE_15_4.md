# PHASE 15.4 — Closed-Loop ESC

**Status: PASS (19/19 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 15.1–15.3 · 14.9 frozen  

---

## Architecture

```
Observation → Decision → Allocation → esc_brake_add → plant
     ↑__________________________________________|
              (closed loop when enabled)
```

**Default: ESC OFF** — passive freeze (3.13 / 8.34 s) preserved.

---

## Gates

| Gate | Result |
|------|--------|
| Enable/disable isolation | OFF ≡ no ESC |
| Zero-intervention (straight) | ΔMz = 0 |
| Disturbance correction | ESC final \|e_r\| ≤ free response |
| Oscillation | 0 Mz sign-flips in step-steer |
| Saturation | ΔMz ≤ 6000, cmd ≤ 1 |
| ABS coexistence | intact |
| util / β / low-speed inhibit | policy holds |
| L/R symmetry | PASS |
| ESC-off & ESC-on longitudinal | **3.13 / 8.34 s** |
| Determinism · stability envelope | PASS |

---

## Notes

- `K_Mz = 4000` is a **baseline candidate**, not a frozen physical constant.
- Strong +ΔMz open-loop disturbances can leave the vehicle in a high-util regime where ESC authority is intentionally limited; ESC must not *worsen* free response.
- Closed-loop performance tuning is **15.4+**, not a reason to reopen 14.9.

---

## Verdict

**PHASE 15.4 — PASS**

```
tag: v1.5.4-closed-loop-esc
report: docs/PHASE_15_4.md
```

### Control stack status

```
15.1 Observation     ✓
15.2 Actuator path   ✓
15.3 Decision logic  ✓
15.4 Closed loop     ✓
```

Further envelope work (split-μ, failure modes) can proceed as 15.5+ without modifying the frozen plant.
