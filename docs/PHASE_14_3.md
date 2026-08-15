# PHASE 14.3 — Crosswind & Sideslip Aerodynamic Coupling

**Status: PASS (15/15 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.2 frozen (`v1.4.2-frozen`)  
**No retuning.** Physics-model capability only.

---

## Mission

Replace the external crosswind disturbance (`st.crosswind * 40 N`) with a physically coupled aerodynamic model driven by relative airflow.

```
Vehicle velocity + wind velocity
             ↓
      relative airflow  V_air = V_v − V_w
             ↓
        β_air = atan2(V_air_y, V_air_x)
             ↓
   Cy = Cyβ·β    Cn = Cnβ·β
      ↙     ↓      ↘
    Fx     Fy      Mz
      ↓     ↓      ↓
         Vehicle
```

---

## Implementation

**Module:** `vehicle_dynamics/aerodynamics/relative_airflow.py`

- `relative_air_velocity(vx, vy, wind_vx, wind_vy)` → `(rel_vx, rel_vy, air_speed, β)`
- `compute_sideslip_aero(...)` → `RelativeAirflowState` with Fx, Fy, Mz, β, q, Cd, Cy, Cn

**Plant wiring:** dual-track path in `simulation.py`

- Wind: `SimulationState.wind_vx` / `wind_vy` (body frame); legacy `crosswind` maps to `wind_vy`
- Longitudinal/downforce use **relative air speed**, not vehicle speed alone
- `ay += Fy_aero / m`, `r_dot += Mz_aero / Iz`
- Coefficients: `Cy_beta = −0.8`, `Cn_beta = −0.15` (linear about β = 0)

---

## Gates

| Gate | Result |
|------|--------|
| Zero-wind 14.2 regression | t100=**3.13 s**, t200=**8.31 s** |
| Relative-airflow calculation | V_air, β computed |
| β calculation | β ≠ 0 under crosswind |
| Aero side-force | Fy ≈ 309 N @ 12 m/s wind, 30 m/s vehicle |
| Aero yaw moment | Mz ≈ 156 N·m |
| Wind symmetry | Fy(+W) = −Fy(−W), Mz symmetric |
| Sign correctness | PASS |
| Coefficient mutation | Cy×2 → Fy×~1.9; Cn×2 → Mz×~2 |
| Aero ON/OFF | Fy_off = 0; drag differs |
| Relative-air-speed authority | wind↑ → \|Fy\|↑ at fixed vx |
| Sideslip authority | \|β\|↑ → \|Fy\|, \|Mz\|↑ |
| Vehicle response | ay_wind = 8.13 vs ay_0 = 0 |
| Historical isolation | 5.36 / 19.77 |
| Deterministic replay ×5 | identical Fy |
| No parameter retuning | mass=1100, P=750, μ=1.15 |

---

## Evidence chain (end-to-end)

```
relative airflow
      ↓
β
      ↓
Cy / Cn
      ↓
Fy / Mz
      ↓
vehicle response (ay, yaw_acc)
```

Demonstrated with symmetry, mutation, ON/OFF, and zero-wind regression.

---

## Frozen 14.2 protection

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar zero-wind | **3.13 s** | **8.31 s** |
| Historical 1400/280 | **5.36 s** | **19.77 s** |

No mass/power/μ/tire/gear changes.

---

## Limitations (accepted)

1. Linear Cy(β), Cn(β) — no high-β saturation or CFD maps.
2. Planar model — α_air reserved, pitch/roll aero not coupled here.
3. Wind specified in body frame for tests; world-frame transform available via `body_wind_from_world`.
4. No active aero, ESC, or yaw controller (out of scope).

---

## Verdict

**PHASE 14.3 — PASS**

```
tag: v1.4.3-crosswind-sideslip-aero
report: docs/PHASE_14_3.md
artifacts: artifacts/phase_14_3/
```
