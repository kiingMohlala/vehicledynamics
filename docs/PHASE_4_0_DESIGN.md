# Phase 4.0 – Dynamic Bicycle Model

**Status:** Design (not yet implemented)

This document freezes the mathematical conventions, equations, interfaces, and validation plan for the 2-DOF dynamic bicycle model before any code is written.

---

## 1. Objectives

- Introduce planar lateral vehicle dynamics on top of the validated combined-slip tire model.
- Support steering input and compute front/rear slip angles.
- Enable constant-radius and step-steer validation cases.
- Preserve the existing tire interface (no changes to `TireModel` or `TireState`).
- Keep longitudinal braking capability available for later combined braking + steering (Phase 4.2).

---

## 2. Coordinate System & Sign Conventions

### Vehicle body frame (ISO 8855 / SAE J670 style)

| Axis | Positive direction |
|------|--------------------|
| +x   | Forward |
| +y   | To the left |
| +z   | Upward |
| Yaw rate r | Positive when rotating about +z (counter-clockwise when viewed from above) |
| Steering angle δ | Positive when front wheels point left |

### State variables (2-DOF)

| Symbol | Description | Unit |
|--------|-------------|------|
| v_y    | Lateral velocity at CG (body frame) | m/s |
| r      | Yaw rate | rad/s |

Optional kinematic outputs:
- ψ (yaw angle)
- X, Y (inertial position)

Longitudinal velocity V_x is treated as a prescribed (quasi-constant) input for pure lateral maneuvers in Phase 4.0.

---

## 3. Geometry

| Symbol | Description |
|--------|-------------|
| a      | Distance from CG to front axle |
| b      | Distance from CG to rear axle |
| L = a + b | Wheelbase |
| m      | Vehicle mass |
| I_z    | Yaw moment of inertia |

Default starting values (to be refined later):

```
m  = 1400 kg
I_z = 2500 kg·m²
a  = 1.2 m
b  = 1.5 m
L  = 2.7 m
```

---

## 4. Slip Angle Definitions

Front and rear axle slip angles (small-angle form acceptable for initial validation; full atan2 form preferred for robustness):

```
α_f = δ - atan2(v_y + a·r, max(|V_x|, V_eps))
α_r =   - atan2(v_y - b·r, max(|V_x|, V_eps))
```

- V_eps is a small regularization speed (reuse tire `v_eps` or a dedicated vehicle parameter).
- Longitudinal slip ratio κ at each axle remains available from the braking module; for pure lateral maneuvers set κ = 0.

---

## 5. Equations of Motion (2-DOF Bicycle)

Lateral force and yaw moment balance:

```
m · (v̇_y + V_x · r) = F_yf + F_yr

I_z · ṙ = a · F_yf - b · F_yr
```

where F_yf and F_yr are the lateral tire forces returned by the combined-slip tire model at the front and rear axles (with the appropriate normal loads and slip angles).

In Phase 4.0 the normal loads may start as static (½ mg front/rear split or a/b static distribution). Dynamic lateral load transfer is deferred to Phase 4.1.

---

## 6. Tire Interface Usage

No changes to the existing tire API:

```python
state_f = tire.longitudinal_lateral_force(
    slip_ratio=kappa_f,      # 0 for pure lateral maneuvers
    slip_angle=alpha_f,
    normal_load=Fz_f
)
state_r = tire.longitudinal_lateral_force(
    slip_ratio=kappa_r,
    slip_angle=alpha_r,
    normal_load=Fz_r
)

Fy_f = state_f.Fy
Fy_r = state_r.Fy
```

Longitudinal forces Fx can be ignored or set to zero for pure lateral tests; they become important in Phase 4.2 (combined braking + steering).

---

## 7. Steering Input

Simple kinematic steering for Phase 4.0:

- δ(t) is an exogenous input (step, ramp, or sine).
- No steering dynamics (power steering lag, etc.) yet.

Typical validation inputs:
- Step steer: δ = 0 → δ_final in one sample
- Constant steer for steady-state circular tests

---

## 8. Proposed Module Structure

```
vehicle_dynamics/
├── lateral/
│   ├── __init__.py
│   ├── parameters.py          # BicycleParameters
│   ├── bicycle.py             # DynamicBicycleModel
│   ├── kinematics.py          # slip angles, path integration
│   ├── simulation.py          # time integration + logging
│   ├── result.py              # LateralSimulationResult
│   ├── validation.py
│   └── visualization.py
└── ...
```

The lateral module will depend on the existing `tire` package but will not modify it.

---

## 9. Validation Plan (Independent, before full-vehicle coupling)

### 9.1 Steady-state circular (constant radius)

- Fixed V_x, fixed δ (or iteratively find δ for a target radius).
- Check:
  - Steady v_y and r exist.
  - Understeer gradient has the correct sign for the chosen mass distribution / cornering stiffnesses.
  - Lateral acceleration a_y ≈ V_x · r.

### 9.2 Step-steer response

- From straight-line running, apply a step in δ.
- Check:
  - Yaw rate and lateral velocity respond smoothly (no numerical blow-up).
  - Final steady state matches the circular-test equilibrium.
  - Reasonable transient (yaw overshoot / damping qualitatively sensible).

### 9.3 Linear-regime consistency

- Small δ and small slip angles → forces approximately linear in α.
- Compare against the classical linear bicycle model analytic yaw-rate gain for a cross-check.

### 9.4 Regression against tire model

- With κ = 0, the tire forces used by the bicycle model must match the already-validated pure-lateral tire curves.

---

## 10. Success Criteria for Phase 4.0

Phase 4.0 is considered complete when:

1. Conventions above are implemented exactly.
2. Independent validation cases (circular + step-steer) pass numerical and qualitative checks.
3. Existing Phase 3 braking/ABS regressions remain unaffected (lateral module is additive).
4. Public tire interfaces are unchanged.

---

## 11. Explicit Non-Goals for Phase 4.0

- No roll degree of freedom / lateral load transfer (Phase 4.1).
- No combined braking + steering scenarios (Phase 4.2).
- No ESC / yaw-moment control (Phase 4.3).
- No suspension compliance or steering compliance.
- No 4-wheel model (remain single-track bicycle).

---

## 12. Open Points to Confirm Before Coding

- [ ] Exact default numerical values for m, I_z, a, b.
- [ ] Whether V_x is held perfectly constant or allowed mild variation.
- [ ] Preferred integrator (RK45 vs fixed-step Euler) for the first implementation.
- [ ] Whether to include a simple aerodynamic drag term in pure lateral tests (recommend: no).

Once these points are confirmed, implementation of `lateral/bicycle.py` can begin under the same validation-first discipline used in Phase 3.
