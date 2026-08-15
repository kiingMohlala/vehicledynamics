# PHASE 14.9.2 — Wheel-Local Slip Angles & Steering–Tire Coupling

**Status: PASS (19/19 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.1 PASS · 14.8 plant frozen  
**No ESC · No retuning**

---

## Mission

```
δ_FL / δ_FR
    ↓
wheel-local velocity (vx_t, vy_t)
    ↓
α_FL / α_FR / α_RL / α_RR
    ↓
Dugoff (κ, α, Fz) → Fx_i / Fy_i
    ↓
ΣFy + ΣMz → ay + yaw
```

---

## Implementation

**`vehicle_dynamics/lateral/slip_angles.py`**

- `compute_wheel_slip_angles(...)` — contact-patch kinematics  
- Sign: `α = atan2(-vy_t, |vx_t|)` so **+δ → +α → +Fy → +ay**  
- Low-speed floor at `v_eps` (no NaN)

**`dual_track_plant.py`** — calls module before Dugoff; independent κ path preserved; dynamic Fz path preserved.

---

## Evidence

| Check | Result |
|-------|--------|
| Ackermann ON | αFL ≠ αFR |
| Left steer | ay > 0, αFL > 0 |
| L/R symmetry | ayL = −ayR |
| μ ×0.5 | Σ\|Fy\| drops |
| Cy mutation | reaches Fy |
| Combined slip | utilization > 0 |
| Zero-steer regression | **3.13 / 8.34 s** |
| Historical | **5.37 s** |

---

## Verdict

**PHASE 14.9.2 — PASS**

```
tag: v1.4.9.2-wheel-local-slip-coupling
report: docs/PHASE_14_9_2.md
```

Next logical card: **14.9.3** steady-state cornering / lateral force & yaw-moment validation (still no ESC).
