# PHASE 14.2H.1 — Runtime Parameter Authority Audit

**Status: PASS (13/13 gates)**  
**Date:** 2026-08-13

No performance tuning. No new physics. Binding only.

---

## Problem addressed

14.2H verified `SimulationConfig`, but `tire_cx` / `tire_cy` (and related plant fields) could still come from `DualTrackConfig` defaults. That is the same class of identity bug as the 1400/280 vs 1100/750 split — smaller, still real.

## Fix

`SimulationConfig` extended with explicit plant fields:

- `tire_cx`, `tire_cy`, `h_cg`, `brake_torque_max`, `track_rear`, `a_fraction`
- `aero_cd`, `aero_cl_front`, `aero_cl_rear`, `aero_frontal_area`

`Simulation.__init__` constructs `DualTrackPlant` and `AeroConfig` **only** from these fields.

`bind_authoritative_hypercar()` copies `TireConfig` / `BrakeConfig` / `GeometryConfig` / `AeroConfigBlock` into `SimulationConfig`.

---

## Provenance table (runtime inspection)

| Parameter | Definition | SimulationConfig | Runtime Plant | Match |
|-----------|------------|------------------|---------------|-------|
| mass | 1100 | 1100 | 1100 | PASS |
| μ | 1.15 | 1.15 | 1.15 | PASS |
| Cx | 100000 | 100000 | [100k×4] Dugoff | PASS |
| Cy | 90000 | 90000 | [90k×4] Dugoff | PASS |
| wheel_radius | 0.33 | 0.33 | 0.33 | PASS |
| drive_split_front | **0.35** | **0.35** | **0.35** | PASS |
| brake_torque_max | 2800 | 2800 | 2800 | PASS |
| h_cg | 0.40 | 0.40 | 0.40 | PASS |
| wheelbase | 2.70 | 2.70 | 2.70 | PASS |
| track_f / track_r | 1.65 / 1.62 | 1.65 / 1.62 | 1.65 / 1.62 | PASS |
| ABS | True | True | True | PASS |
| engine peak torque | ≈1591.5 | 1591.5 | 1591.5 | PASS |
| final_drive | 3.9 | 3.9 | 3.9 | PASS |
| gear ratios | standard 6-spd | via final_drive | [3.5…0.85] | PASS |
| aero Cd / Cl_f / Cl_r | 0.34 / −0.55 / −0.85 | same | same | PASS |

**AWD WOT step:** T_front=700 N·m, T_rear=1300 N·m, split_rt=**0.350** (exact policy, not merely `> 0`).

---

## Gates

| Gate | Result |
|------|--------|
| runtime_provenance_complete | 18/18 rows match |
| runtime_awd_authority | split=0.35, T_f>0, T_r>0 |
| runtime_tire_parameter_authority | all four Dugoff Cx/Cy/μ |
| runtime_brake_parameter_authority | Tmax=2800, ABS on |
| abs_runtime_pressure_authority | std=0.32, min=0.15 |
| runtime_powertrain_parameter_authority | peak_tq=1591.5, FD=3.9, P=750 |
| runtime_aero_parameter_authority | Cd/Cl bound |
| aero_dynamics_authority | ax differs ON vs OFF |
| runtime_geometry_parameter_authority | h_cg, L, tracks |
| no_authority_fallback | chain closed |
| historical_regression | t100=5.36, t200=19.77 |
| authoritative_hypercar_replay | t100=3.13, t200=8.31, mass=1100, P=750 |
| identity_not_historical | plant_mass=1100 |

---

## Regression

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Historical 1400/280 | **5.36 s** | **19.77 s** |
| Hypercar 1100/750 | **3.13 s** | **8.31 s** |

Unchanged vs 14.2H (no retuning).

---

## Remaining limitations

1. Gear ratio *set* is still the library `default_ratios(final_drive)` — not a custom ratio vector on `VehicleDefinition`. Final drive **is** bound; individual ratios are the shared sequential defaults (documented, not a silent mass/power substitution).
2. Crosswind remains an external disturbance model.
3. Energy residual accounting remains partial (14.2F status unchanged).

---

## Verdict

**PHASE 14.2H.1 — PASS**

```
VehicleDefinition → SimulationConfig → DualTrackPlant / Dugoff / ABS / Aero / Powertrain
```

with zero unexplained parameter substitutions on the audited critical set.

```
validation: vehicle_dynamics/demonstration/validation_14_2H1.py
report: docs/PHASE_14_2H1.md
```
