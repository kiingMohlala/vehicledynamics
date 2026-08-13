# PHASE 14.2H — Authoritative Vehicle Binding & Full Revalidation

**Status: PASS (30/30 gates)**  
**Date:** 2026-08-13  
**Tag:** `v1.4.2h-authoritative-vehicle`

---

## Two identities (never mixed)

| Identity | Mass | Power | Role |
|----------|------|-------|------|
| **HISTORICAL DEMONSTRATOR** | **1400 kg** | **280 kW** | Frozen 14.2D/E artifact (`historical_demonstrator_config()`) |
| **AUTHORITATIVE HYPERCAR** | **1100 kg** | **750 kW** | Project reference (`bind_authoritative_hypercar()`) |

---

## Binding path (authoritative)

```
VehicleDefinition (1100 kg, 750 kW, AWD, high DF)
      → DigitalTwin
      → SimulationConfig (mass, power, μ, r, FD, ABS, split, aero…)
      → Simulation / Powertrain / DualTrackPlant / Dugoff / ABS / Aero
      → Telemetry + config_fingerprint
```

Provenance examples:

| Parameter | Binding |
|-----------|---------|
| mass | `MassProperties.total_mass_kg` → `SimulationConfig.mass` = **1100** (fuel=0) |
| peak_power_kw | `PowertrainConfigBlock.peak_power_kw` → `SimulationConfig.peak_power_kw` = **750** |
| peak_torque_nm | derived P/ω @ 4500 rpm ≈ **1591 N·m** |
| μ | `TireConfig.mu` → `SimulationConfig.mu_tire` = **1.15** |
| wheel_radius | `TireConfig.radius_m` → **0.33 m** |
| final_drive | `PowertrainConfigBlock` → **3.9** |
| drive_split_front | AWD policy → **0.35** |
| abs_enabled | `BrakeConfig` → **True** |

Fingerprint: `d840596e4dbd5600`

Runtime identity gate inspects **SimulationConfig**, not the brief string — **PASS**.

---

## Historical regression (unchanged)

| Metric | Expected | Replay |
|--------|----------|--------|
| 0–100 | 5.36 s | **5.36 s** |
| 0–200 | 19.77 s | **19.77 s** |
| mass / power | 1400 / 280 | **1400 / 280** |

`validation_14_2E.frozen_cfg()` remains historical-only — **not** used for the hypercar.

---

## Authoritative hypercar results (simulation — not tuned)

| Metric | Value | Source |
|--------|-------|--------|
| 0–50 | **1.77 s** | simulation |
| 0–100 | **3.13 s** (×3, std 0) | simulation |
| 0–200 | **8.30 s** | simulation |
| peak ax | 11.65 m/s² | simulation |
| Dry 100–0 | 2.26 s, ABS active | simulation |
| Wet 100–0 | 4.45 s | simulation |
| Split-μ | asymmetric Fx, yaw_acc=1.57 | simulation |

**No parameter was changed to hit a target time.** Result is the plant output for 1100 kg / 750 kW.

Identity separation: hypercar t100 (3.13 s) ≠ historical (5.36 s) — proves different vehicles.

---

## Handling / braking / aero / powertrain

| Gate | Result |
|------|--------|
| Handling chain steer→α→Dugoff Fy→ay / Mz | PASS |
| Cy authority (90k→45k reduces \|ay\|) | PASS |
| Slalom / DLC / figure-eight | PASS |
| ABS pressure modulation | std_P=0.30, min_P=0.15 |
| Aero ON vs OFF @ 50 m/s | ax −7.53 vs −6.27 |
| Powertrain 750 kW bound at runtime | PASS |

---

## Energy

Ledger run on hypercar (15 s WOT): E_engine > E_vehicle; residual_frac still reflects incomplete shift/clutch instrumentation (same limitation as 14.2F). **Not labelled CLOSED.** Gate requires no free energy only — PASS.

---

## Limitations

1. DualTrack `tire_cx`/`tire_cy` still taken from `DualTrackConfig` defaults (100k/90k), chosen to match `TireConfig` values — not yet a separate runtime field on `SimulationConfig`.
2. Energy residual not fully shaft-instrumented → PARTIAL energy accounting (honest).
3. Crosswind remains external disturbance (unchanged honesty from 14.2F).
4. Historical `frozen_cfg()` name retained for 14.2E reproducibility; new code must use `bind_authoritative_hypercar()` or `historical_demonstrator_config()` explicitly.

---

## Verdict

**PHASE 14.2H — PASS**

The vehicle being simulated for the hypercar brief is the vehicle defined by the authoritative binding (1100 kg / 750 kW). Historical 14.2D/E evidence remains reproducible under its frozen 1400/280 configuration.

```
module: vehicle_dynamics/demonstration/vehicle_binding.py
validation: vehicle_dynamics/demonstration/validation_14_2H.py
report: docs/PHASE_14_2H.md
```
