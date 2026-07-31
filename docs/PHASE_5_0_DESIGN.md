# Phase 5.0 – Dual-Track Vehicle Model (4-Wheel)

**Status:** Design (not yet implemented)

This document freezes the architecture, conventions, and validation plan for the dual-track planar vehicle model before any code is written.

---

## 1. Objectives

- Replace the single-track (bicycle) representation with a **four-wheel planar model**.
- Independent normal loads, slip ratios, and slip angles at FL / FR / RL / RR.
- True yaw moments from both lateral **and** longitudinal tire forces.
- Preserve combined-slip Dugoff tire API (unchanged).
- Provide the geometric foundation for later ESC, brake vectoring, and torque distribution.
- Regression: when left/right inputs are symmetric, behaviour should closely match the validated Phase 4.2 bicycle results.

---

## 2. Scope

### In scope
- Planar rigid body with states: Vx, vy, r, ψ, X, Y
- Four wheels with independent:
  - κ_ij (longitudinal slip)
  - α_ij (lateral slip angle)
  - Fz_ij (normal load)
  - ω_ij (spin speed, if wheel dynamics enabled)
- Steering geometry: front wheels steered by δ (Ackermann optional; equal δ first)
- Brake torque distribution to four wheels (reuse Phase 3 bias + optional ABS per wheel or per axle)
- Static or Phase 4.1 diagnostic load transfer → actual Fz_ij used by tires (first time load transfer **feeds** the tire model)
- Yaw equation including track-width contributions from Fx and Fy

### Explicitly out of scope (later phases)
- Full suspension compliance / camber dynamics
- Dynamic roll DOF (roll rate, roll inertia) — optional algebraic roll angle only if needed for load transfer
- ESC controller (Phase 5.1+)
- Active differentials / torque vectoring algorithms
- Aerodynamics
- Tire relaxation length

---

## 3. Geometry

```
        δ   δ
      FL----FR          track_f
       |    |
       | CG |           a (front), b (rear)
       |    |
      RL----RR          track_r
```

| Symbol | Meaning | Default |
|--------|---------|---------|
| a | CG → front axle | 1.2 m |
| b | CG → rear axle | 1.5 m |
| track_f | Front track | 1.55 m |
| track_r | Rear track | 1.55 m |
| h_cg | CG height (load transfer) | 0.55 m |
| R | Wheel radius | 0.33 m |

Wheel positions relative to CG:
```
FL: (+a, +track_f/2)
FR: (+a, -track_f/2)
RL: (-b, +track_r/2)
RR: (-b, -track_r/2)
```
(Sign of y: +y to the left.)

---

## 4. States

| State | Description |
|-------|-------------|
| Vx, vy, r | Planar body velocities |
| ψ, X, Y | Orientation and position |
| ω_fl, ω_fr, ω_rl, ω_rr | Wheel spin speeds |

---

## 5. Slip Definitions (per wheel)

Velocity of wheel centre in body frame:
```
Vx_ij = Vx - r * y_ij
Vy_ij = vy + r * x_ij
```

Steered front wheels: rotate (Vx_ij, Vy_ij) into wheel frame by δ.

```
κ_ij = (V_x,wheel - ω_ij * R) / max(|V_x,wheel|, v_eps)
α_ij = atan2(V_y,wheel, max(|V_x,wheel|, v_eps))   # with sign convention consistent with Phase 4
```

---

## 6. Equations of Motion

```
m * (V̇x - vy * r) = Σ Fx_body_ij
m * (v̇y + Vx * r) = Σ Fy_body_ij
I_z * ṙ = Σ (x_ij * Fy_body_ij - y_ij * Fx_body_ij)
```

Tire forces are computed in the wheel frame via the existing combined-slip API, then transformed into the body frame (accounting for steer angle on the front axle).

---

## 7. Normal Loads

**First use of load-transfer feedback:**

```
Fz_ij = static_ij + lateral_transfer_ij + longitudinal_transfer_ij
```

- Lateral transfer: reuse Phase 4.1 formulas (now applied to tire Fz).
- Longitudinal transfer: m * ax * h_cg / L split front/rear (same idea as Phase 3 weight transfer).
- Clamp with axle-preserving logic from Phase 4.1.

---

## 8. Module Structure

```
vehicle_dynamics/
├── dual_track/
│   ├── parameters.py          # DualTrackParameters
│   ├── kinematics.py          # wheel velocities, slips, steer transform
│   ├── normal_loads.py        # static + lateral + longitudinal transfer
│   ├── simulation.py          # DualTrackVehicleModel
│   ├── result.py
│   ├── validation.py
│   └── __init__.py
└── ...
```

Compose existing tire, brake, and ABS modules; do not rewrite them.

---

## 9. Validation Plan

1. **Symmetric pure steering**  
   Equal left/right → yaw rate / vy within tolerance of Phase 4.0/4.2 bicycle.

2. **Symmetric pure braking**  
   Straight-line stop; lateral states near zero; distance comparable to Phase 3/4.2.

3. **Load-transfer feedback**  
   Steady corner: outer wheels higher Fz than inner; totals conserved.

4. **Yaw from Fx**  
   Asymmetric brake (e.g. left side only) produces yaw moment of the correct sign.

5. **Trail braking**  
   Same qualitative checks as Phase 4.2; utilization ≤ 1; no instability.

6. **Numerical robustness**  
   No NaN/Inf across mu and speed sweeps.

---

## 10. Success Criteria

- Four independent tire calls per step using the frozen combined-slip API.
- Yaw equation includes both Fx and Fy contributions via track geometry.
- Load-transfer feedback active and validated.
- Symmetric-input regressions against Phase 4.2 pass within agreed tolerance.
- No ESC logic yet.

---

## 11. Open Points to Confirm Before Coding

1. **Ackermann steering** – equal δ on FL/FR first, or simple Ackermann from the start?
2. **ABS granularity** – per-wheel or per-axle for the first implementation?
3. **Longitudinal load transfer** – include from day one, or lateral-only feedback first?
4. **Integrator** – continue RK45 with the larger state vector?
5. **Regression tolerance** vs bicycle model (e.g. 5–10% on steady yaw rate).

Once confirmed, implementation can begin under the same validation-first workflow.
