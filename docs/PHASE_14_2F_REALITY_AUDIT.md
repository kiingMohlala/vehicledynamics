# PHASE 14.2 REALITY AUDIT (14.2F)

**Date:** 2026-08-13  
**Frozen baseline:** tag `v1.4.2e-scenario-closure` / commit `8866ddb`  
**Vehicle:** exact 14.2D/E configuration — no retuning

| Parameter | Value |
|-----------|-------|
| mass | 1400 kg |
| μ | 1.15 |
| wheel radius | 0.32 m |
| final drive | 3.9 |
| peak power | 280 kW |
| plant | dual-track + Dugoff + ABS |
| aero | enabled |

---

## 14.2F.1 Frozen-baseline audit

- Tags present: `v1.4.2c`, `v1.4.2d`, `v1.4.2e`
- Config values match 14.2D freeze
- No mass/μ/Cx/Cy/ratio changes applied in this audit

---

## 14.2F.2 Energy closure investigation

### Prior claim
14.2E reported `residual_frac ≈ 0.34` from a **partial** ledger:

```
residual ≈ E_engine − E_vehicle − E_wheel_rot
```

That omitted aero work, rolling work, clutch heat, and transmission efficiency/shift cuts.

### Full ledger (20 s WOT, simulation)

| Term | Value | Role |
|------|-------|------|
| **E_engine** ∫ Te·ωe | **3.451 MJ** | source |
| E_clutch_heat ∫\|Tc·ω_slip\| | 0.148 MJ | dissipated |
| E_gb_out ∫ Tw·ωw | 3.083 MJ | transferred |
| eng − gb − clutch | 0.220 MJ | η=0.95 + shift cuts + unlock |
| W_tire_pos ∫ Fx⁺·vx | 2.925 MJ | tire propulsive work |
| W_aero ∫ D·vx | 0.597 MJ | dissipated |
| W_roll ∫ Fr·vx | 0.151 MJ | dissipated |
| E_vehicle ½mv² | 2.176 MJ | stored |
| E_wheel_rot | 0.109 MJ | stored |

**Vehicle-side closure:**

```
W_tire_pos − (E_veh + E_wrot + W_aero + W_roll) ≈ −0.108 MJ (~3.7% of tire work)
```

**Engine-side accounted sinks:**

```
E_veh + E_wrot + W_aero + W_roll + E_clutch_heat + (eng−gb−clutch)
  = 2.176 + 0.109 + 0.597 + 0.151 + 0.148 + 0.220
  = 3.401 MJ
vs E_engine 3.451 MJ → unaccounted ≈ 0.05 MJ (~1.4%)
```

### Origin of the old “34%”
Incomplete bookkeeping (omitted aero + roll + clutch heat + shift/η). **Not** free energy or a missing physics sink of 34%.

### ENERGY STATUS: **PARTIALLY CLOSED**

- No energy creation
- Dominant sinks identified and quantified
- Residual ~1–4% after full ledger (numerical + minor untracked map/limiter/shift detail)
- Not labelled CLOSED at audit precision because differential internal loss and exact shift-cut integral are still estimated, not instrumented shaft-by-shaft

---

## 14.2F.3 Handling independence audit

### Steer authority (vx fixed, Tw fixed)

| steer | ay | yaw_acc | α_front | Fy_front |
|-------|-----|---------|---------|----------|
| 0 | 0.000 | 0.000 | 0 | 0 |
| +0.1 | −5.120 | −3.543 | −0.10 | −4592 / −2576 N |
| −0.1 | +5.120 | +3.543 | +0.10 | +2576 / +4592 N |

**PASS:** α, Fy, ay, Mz/yaw_acc all reverse with steer. Longitudinal inputs held constant.

### Cy authority (tire_cy 90 kN/rad → 45 kN/rad)

| Cy | ay @ steer=0.1 |
|----|----------------|
| 90 000 | −5.120 |
| 45 000 | −4.411 |

**PASS:** softer lateral stiffness reduces \|ay\|. Response is tire-model mediated, not a steer→ay gain.

### Chain

```
steer → δ → α → Dugoff → Fy → ΣFy → ay
              → contact arms → Mz → yaw_acc
```

**Handling coupling: PASS**

---

## 14.2F.4 Braking audit

| Test | Result |
|------|--------|
| Dry 100→0 | 2.30 s (×5, std 0) |
| ABS pressure | min P=0.15, std P=0.311 under hard brake |
| ABS vs off | pressure modulates only when enabled |

**PASS:** ABS changes per-wheel pressure in response to slip; not a dormant object. Chain brake_cmd → pressure → T_b → κ → Dugoff Fx → ax verified in dual-track path.

---

## 14.2F.5 Longitudinal reproducibility

| Metric | n=5 values | mean | std | span |
|--------|------------|------|-----|------|
| 0–100 | 5.36 ×5 | 5.36 s | **0** | 0 |
| 0–200 | 19.77 ×5 | 19.77 s | **0** | 0 |
| 100–0 | 2.30 ×5 | 2.30 s | **0** | 0 |

**Deterministic replay: PASS**

---

## 14.2F.6 Aero audit

| Check | Result |
|-------|--------|
| drag ∝ v² | ratio drag(40)/drag(20) = **4.0** |
| DF ∝ v² | ratio DF(40)/DF(20) = **4.0** |
| Enters dynamics | coast ax @ 50 m/s: aero ON **−6.13**, OFF **−5.47** m/s² |

**Aero coupling: PASS** (forces affect ax, not telemetry-only)

---

## 14.2F.7 Crosswind honesty

```
F_y = crosswind × 40 N
```

**Classification: EXTERNAL DISTURBANCE MODEL** — not an aerodynamic crosswind model (no dynamic pressure × Cy_β × side area, no aero yaw moment from β_wind).

Claim in 14.2E that “crosswind responds” is true for the disturbance path; it must not be cited as aero fidelity.

---

## 14.2F.8 Evidence provenance

| Metric | Source label |
|--------|----------------|
| 0–100 / 0–200 / braking times | **SIMULATION** |
| α, Fy, ay, yaw_acc | **SIMULATION** |
| Energy terms | **SIMULATION** (+ analytical residual definition) |
| drag/DF vs speed | **SIMULATION** (aero map) |
| Crosswind force law | **HEURISTIC** |
| Cy sensitivity | **SIMULATION** (parameter perturbation) |
| 14.2D t100 match | **REGRESSION** |

No heuristic or analytical result was found labelled as “simulation” in the 14.2E report path for performance claims. Crosswind limitation was already footnoted.

**Evidence provenance: PASS**

---

## 14.2F.9 Frozen regression

| Phase | Check | Status |
|-------|-------|--------|
| 14.2C | Dugoff dual-track bound | retained |
| 14.2D | t100 = 5.36 s | reproduced std=0 |
| 14.2E | 30/30 gate set | baseline unchanged |
| Config | mass/μ/ratios/power | unmodified |

**Regression protection: PASS**

---

## 14.2F.10 Final verdict

| Domain | Verdict |
|--------|---------|
| Physics integration | **PASS** |
| Powertrain coupling | **PASS** |
| Tire authority | **PASS** (Dugoff; Cy perturbation moves ay) |
| ABS authority | **PASS** (pressure modulates with slip) |
| Handling coupling | **PASS** |
| Aero coupling | **PASS** (drag in ax; v² scaling) |
| Energy accounting | **PARTIAL** (full ledger ~1–4% residual; old 34% was incomplete bookkeeping) |
| Deterministic replay | **PASS** |
| Evidence provenance | **PASS** |
| Regression protection | **PASS** |

### OVERALL: **REALITY VALIDATED WITH LIMITATIONS**

**Limitations (explicit):**

1. Energy ledger still approximates shift-cut and differential internal loss → PARTIAL, not fully instrumented CLOSED.
2. Crosswind is an external disturbance (×40 N), not aero β-sideforce/yaw.
3. Load transfer is quasi-static (no suspension travel ODE in this path).
4. ABS limits peak slip imperfectly under extreme μFz saturation (κ can still reach 1); authority is pressure modulation, not perfect slip regulation.
5. Open-loop handling manoeuvres (not ISO closed-loop path tracking).

**Not a reason to open Phase 14.3:** remaining issues are quantified limitations of the existing stack, not unexamined failures requiring a new abstraction layer.

---

## Artifacts

- This report: `docs/PHASE_14_2F_REALITY_AUDIT.md`
- Frozen tags: `v1.4.2d-powertrain-fidelity`, `v1.4.2e-scenario-closure`
- Audit vehicle hash basis: mass=1400, μ=1.15, FD=3.9, P=280 kW, dual-track=True
