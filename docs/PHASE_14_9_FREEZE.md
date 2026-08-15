# PASSIVE VEHICLE DYNAMICS — FROZEN

**Status:** CLOSED  
**Date:** 2026-08-15  

---

## Tags

| Role | Tag |
|------|-----|
| Foundation | `v1.4.8-coupled-plant-validated` |
| Passive closure | `v1.4.9.9-combined-slip-brake-corner` |
| Freeze marker | `v1.4.9-passive-dynamics-frozen` |

---

## Progression

```
14.8  Coupled plant
 │
 ├─ 14.9.1  Steering / Ackermann
 ├─ 14.9.2  Wheel-local slip
 ├─ 14.9.3  Steady-state cornering
 ├─ 14.9.4  Transient lateral + yaw
 ├─ 14.9.5  Mechanical ARB
 ├─ 14.9.6  Hydraulic ARB
 ├─ 14.9.7  Roll-stiffness distribution
 ├─ 14.9.8  Understeer / oversteer + yaw stability
 └─ 14.9.9  Combined braking + cornering
          │
          ▼
   PASSIVE DYNAMICS CLOSED
          │
          ▼
       15.x CONTROL
```

---

## Coupled chain (validated)

```
steering → α_i → Fz redistribution (ARB/roll)
                → Dugoff(κ, α, Fz) → Fx + Fy
                → ΣFx → ax
                → ΣFy → ay
                → ΣMz → yaw_acc → r
```

---

## Frozen longitudinal reference

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | **3.13 s** | **8.34 s** |
| Historical | **5.37 s** | **19.81 s** |

## Handling summary

- Natural balance: **understeer**
- Yaw-stable to tire limit
- Combined-slip friction budget validated (β ≤ ~1)
- Mechanical + hydraulic ARB validated

---

## Forbidden without a new phase

- Retuning vehicle identity (mass, power, μ, gears, tire Cx/Cy, aero)
- ESC / yaw controller
- Active roll control / active ARB pressure
- Torque vectoring
- ABS parameter retuning
- New tire model or CFD claims

**Exception:** genuine bug fixes that restore already-validated behaviour (with regression evidence).

---

## Next boundary

**15.1 — ESC Observability & Reference Yaw Model**

```
vehicle state → β / r / ay / δ / vx
             → r_ref
             → yaw-rate error
             → ESC decision variables
```

No brake intervention in 15.1.  
No torque vectoring.  
Prove observability and the reference model before any actuator command.
