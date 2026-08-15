# PHASE 14.4 — Dynamic Load Transfer & Wheel-Load Authority

**Status: PASS (17/17 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.3 PASS  
**No retuning** of frozen 14.2 mass/power/μ/tires/gears.

---

## Mission

Make wheel normal loads explicit, authoritative, and coupled into Dugoff:

```
ax, ay, geometry, mass, aero downforce
              ↓
      compute_wheel_loads()
              ↓
   Fz_FL, Fz_FR, Fz_RL, Fz_RR
              ↓
         Dugoff (Fx, Fy)
              ↓
        vehicle response
```

---

## Implementation

**`vehicle_dynamics/lateral/load_transfer.py`**

- `compute_wheel_loads(...)` — full model:
  - Static: `Fz_f = mg·b/L`, `Fz_r = mg·a/L`
  - Long: `ΔFz_f = −m·ax·h/L`
  - Aero: `downforce_front` / `downforce_rear` (from Cl split, not 50/50 hard-code)
  - Lateral: `m·ay·h/track · chi_f`
  - Unload floor: `Fz_min = 50 N` (documented clamp)

**`DualTrackPlant`**

- Uses `compute_wheel_loads` each step
- Accepts `downforce_front` / `downforce_rear` from 14.3 aero state
- `chi_f`, `Fz_min` on `DualTrackConfig` from `SimulationConfig`
- Diagnostics: `Fz_FL..RR`, `min_Fz`, `max_Fz`, `lt`

---

## Gates (17/17)

| Gate | Result |
|------|--------|
| load_transfer_authority | mass, h_cg, tracks, WB, chi_f → plant |
| static_wheel_load_distribution | L/R equal; ΣFz = mg |
| longitudinal_transfer_symmetry | +ax ↔ −ax (ΔFz magnitude) |
| lateral_transfer_symmetry | +ay ↔ −ay |
| wheel_load_conservation | ΣFz = mg (zero aero) |
| tire_fz_coupling | h_cg×2 → dFz×1.78 |
| aero_fz_coupling | ON ΣFz=14767 vs OFF 10791 |
| combined_braking_cornering | Fz transfer under brake+steer |
| wheel_unloading_behavior | min_Fz ≥ 50, no NaN |
| hcg / track / wheelbase / aero mutations | all reach runtime Fz |
| negative_default_fallback | poisoned DualTrack defaults ignored |
| historical_isolation | 5.36 / 19.65 |
| zero_wind_regression | 3.13 / 8.30 |
| deterministic_replay ×5 | identical Fz |

---

## Regression (frozen baseline protected)

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | **3.13 s** | **8.30 s** |
| Historical | **5.36 s** | **19.65 s** |

Within established tolerance; no retuning.

---

## Limitations (accepted)

1. Load transfer remains **quasi-static** (no suspension travel ODE / roll inertia).
2. `Fz_min = 50 N` floor prevents singular tire model — physical unload is reported via clamp flag.
3. Lateral transfer uses roll-stiffness proxy `chi_f = 0.55` (not a full suspension model).

---

## Verdict

**PHASE 14.4 — PASS**

```
tag: v1.4.4-dynamic-load-transfer
report: docs/PHASE_14_4.md
artifacts: artifacts/phase_14_4/
```
