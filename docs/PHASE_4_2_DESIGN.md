# Phase 4.2 – Combined Braking + Steering

**Status:** Design (not yet implemented)

This document freezes the scope, interfaces, and validation plan for combined longitudinal + lateral maneuvers before any code is written.

---

## 1. Objectives

- Couple the validated Phase 3 braking plant with the validated Phase 4.0 bicycle model.
- Exercise the combined-slip tire model with **non-zero κ and α simultaneously**.
- Support trail-braking and constant-speed cornering-with-brake scenarios.
- Preserve existing Phase 3 and Phase 4.0 regressions when the unused axis is idle (κ=0 or α=0).
- Do **not** introduce ESC, torque vectoring, or dual-track dynamics yet.

---

## 2. Scope

### In scope
- Single-track (bicycle) planar model with:
  - Longitudinal speed Vx as a **dynamic state** (no longer forced constant)
  - Lateral velocity vy, yaw rate r
- Steering input δ(t) and brake pedal input p_brake(t)
- Combined-slip tire forces (Fx, Fy) at front and rear axles
- Optional ABS on longitudinal slip (reuse Phase 3.2 controller)
- Static axle normal loads (Phase 4.1 diagnostics may still be logged)
- Validation cases:
  - Pure braking (δ=0) recovers Phase 3 behaviour within tolerance
  - Pure steering (p_brake=0) recovers Phase 4.0 behaviour within tolerance
  - Trail braking (brake while turning)
  - Simple understeer / oversteer observation under combined inputs

### Explicitly out of scope
- Lateral load-transfer feedback into tire Fz (remains diagnostic)
- Dual-track / left-right individual braking
- ESC / yaw-moment control (Phase 4.3)
- Roll / pitch dynamics
- Aerodynamics
- Driveline / engine torque

---

## 3. States

| State | Description | Unit |
|-------|-------------|------|
| Vx    | Longitudinal velocity | m/s |
| vy    | Lateral velocity | m/s |
| r     | Yaw rate | rad/s |
| ψ     | Yaw angle | rad |
| X, Y  | Inertial position | m |
| ωf, ωr | Optional wheel speeds (if ABS / longitudinal slip is active) | rad/s |

---

## 4. Equations of Motion (planar)

```
m · (V̇x - vy · r) = Fx_f + Fx_r
m · (v̇y + Vx · r) = Fy_f + Fy_r
I_z · ṙ = a · Fy_f - b · Fy_r + (moment from longitudinal forces if track is modelled; zero for pure bicycle)
```

For the single-track model the yaw contribution of Fx is typically neglected unless a dual-track geometry is introduced. Phase 4.2 keeps the pure bicycle yaw equation (Fy terms only) unless explicitly extended.

---

## 5. Slip Definitions

Reuse existing definitions:

```
κ_i = (Vx - ω_i · R) / max(|Vx|, v_eps)     # or axle equivalent
α_f = δ - atan2(vy + a·r, max(|Vx|, v_eps))
α_r =   - atan2(vy - b·r, max(|Vx|, v_eps))
```

Tire call (unchanged API):

```python
state_f = tire.longitudinal_lateral_force(κ_f, α_f, Fz_f)
state_r = tire.longitudinal_lateral_force(κ_r, α_r, Fz_r)
Fx_f, Fy_f = state_f.Fx, state_f.Fy
Fx_r, Fy_r = state_r.Fx, state_r.Fy
```

---

## 6. Brake Coupling

Reuse Phase 3 brake torque + optional ABS:

```
T_f_des, T_r_des = brake_torque.desired(pedal)
# optional ABS modulation on each axle / wheel
# wheel dynamics → ω → κ
```

For the first implementation, a simplified axle-level longitudinal slip model is acceptable (effective axle κ derived from a single equivalent wheel speed per axle).

---

## 7. Module Structure

```
vehicle_dynamics/
├── combined/                    # or extend lateral/ + braking/
│   ├── parameters.py
│   ├── simulation.py           # CombinedVehicleModel
│   ├── result.py
│   ├── validation.py
│   └── scenarios.py            # trail-brake, step-steer+brake helpers
└── ...
```

Prefer **composition** of existing `DynamicBicycleModel` concepts + `BrakeSimulation` pieces over rewriting either subsystem.

---

## 8. Validation Plan

1. **Pure braking regression (δ = 0)**  
   Stopping distance / slip behaviour remains consistent with Phase 3 within tolerance.

2. **Pure steering regression (pedal = 0)**  
   Step-steer and circular tests recover Phase 4.0 within tolerance.

3. **Trail braking**  
   Brake while holding constant steer:  
   - Vx decreases  
   - yaw rate / path curvature respond continuously  
   - no NaN / instability  
   - utilization rises but stays ≤ 1

4. **Combined-slip activity**  
   Confirm |κ| and |α| are simultaneously non-zero during trail braking and that both Fx and Fy are non-zero.

5. **Symmetry**  
   +δ with braking vs −δ with braking produces mirrored lateral states.

---

## 9. Success Criteria

Phase 4.2 is complete when:

- Combined state equations are implemented with Vx dynamic.
- Tire API is unchanged and receives non-zero (κ, α) pairs.
- Pure-braking and pure-steering regressions pass.
- At least one trail-braking scenario runs stably and is documented.
- No ESC or dual-track physics has been introduced.

---

## 10. Open Points to Confirm Before Coding

1. **Wheel dynamics fidelity** – full per-axle wheel inertia + ABS, or simplified kinematic κ from brake force?
2. **Yaw moment from Fx** – keep pure bicycle (Fy only) or add a simple dual-track yaw contribution?
3. **Integrator** – continue RK45 for the combined system?
4. **Default scenario set** for the first validation report.

Once confirmed, implementation can begin under the same validation-first workflow.
