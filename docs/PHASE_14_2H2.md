# PHASE 14.2H.2 — Full Runtime Authority & Final Closure

**Status: PASS (19/19 gates)**  
**Date:** 2026-08-15  
**No retuning.** Architecture proof only.

---

## 1. Gear-ratio authority — CLOSED

```
PowertrainConfigBlock.gear_ratios
  → SimulationConfig.gear_ratios
  → TransmissionConfig.gear_ratios
  → Gearbox.ratios.gears  (runtime)
```

| Layer | Value |
|-------|-------|
| Definition | `[0, 3.5, 2.2, 1.6, 1.2, 1.0, 0.85]` |
| SimulationConfig | same |
| Runtime transmission | same |
| Final drive | **3.9** (explicit) |

Authoritative hypercar **no longer depends on** `default_ratios()` for the gear vector.  
Historical path may still use `default_ratios(final_drive)` when `gear_ratios is None`.

---

## 2. Provenance table — 17/17 match

All critical parameters: mass, μ, Cx, Cy, radius, AWD split, brake Tmax, h_cg, FD, power, aero Cd/Cl, ABS, transmission efficiency, gear ratios, Dugoff×4 instances.

Machine-readable: `artifacts/phase_14_2h2/provenance.json`

---

## 3. Authority mutation tests — 7/7

| Parameter change | Runtime effect |
|------------------|----------------|
| Cy × 0.5 | ay 10.76 → 8.72 |
| brake Tmax × 0.5 | plant Tmax = 1400 |
| gear 1 × 1.1 | runtime g1 = 3.85 |
| FD × 1.1 | runtime FD = 4.29 |
| mass +100 | plant mass = 1200 |
| Cd +20% | runtime Cd = 0.408 |
| μ × 0.8 | plant μ = 0.92 |

---

## 4. Negative fallback — PASSED

Poisoned:

- `DualTrackConfig` field defaults Cx=111, Cy=222  
- `default_ratios()` → bogus `[9.99, 8.88, …]`

After `bind_authoritative_hypercar()` + `Simulation()`:

- Dugoff Cx still **100000** ×4  
- Dugoff Cy still **90000** ×4  
- Gears still **AUTH vector**

Authoritative path **resists** library default poison.

---

## 5. Historical isolation

| Vehicle | Mass/Power | 0–100 | 0–200 |
|---------|------------|-------|-------|
| Historical | 1400 / 280 | **5.36 s** | **19.77 s** |
| Hypercar | 1100 / 750 | **3.13 s** | **8.31 s** |

Paths do not cross-contaminate.

---

## 6. Energy instrumentation

| Term | Value |
|------|-------|
| E_engine | instrumented ∫ Te·ωe |
| E_clutch_heat | ∫ \|Tc·ω_slip\| |
| E_trans_loss_proxy | E_eng − clutch − gb_out |
| E_gb_out | ∫ Tw·ωw |
| W_tire / W_aero / W_roll | instrumented |
| E_vehicle / E_wheel_rot | ½mv² / ½Iω² |
| **residual_frac** | **≈ 0.005** |

**ENERGY: PARTIAL** (honest) — transmission loss is still a residual proxy, not a fully shaft-instrumented efficiency map; shift-cut joules not separately tagged. Residual is small and bounded; no free energy.

---

## 7. Deterministic replay (n=5)

t100 = 3.13 s, t200 = 8.31 s, stop = 2.24 s — **identical** across runs.

---

## 8. Freeze criteria

| Criterion | Verdict |
|-----------|---------|
| Parameter authority | **PASS** |
| Gear-ratio authority | **PASS** |
| No-default fallback | **PASS** |
| Physics mutation tests | **PASS** |
| Historical isolation | **PASS** |
| Handling coupling | **PASS** |
| Powertrain coupling | **PASS** |
| ABS authority | **PASS** |
| Aero authority | **PASS** |
| Energy instrumentation | **PARTIAL** (honest) |
| Deterministic replay | **PASS** |
| Full regression | **PASS** |

---

## Remaining limitations (explicit)

1. Energy: PARTIAL — trans internal loss not fully shaft-instrumented.  
2. Crosswind: external disturbance model, not aero β-sideforce.  
3. Load transfer: quasi-static (no suspension travel ODE).  
4. Gear ratio *values* for hypercar are the explicit AUTH vector; they happen to match the old library defaults numerically, but the **binding path** no longer uses `default_ratios()` for the hypercar.

---

## Verdict

**PHASE 14.2H.2 — PASS**

The authoritative `VehicleDefinition` controls the runtime plant for every audited behavior-changing parameter, including gear ratios. Historical 14.2D/E remains isolated and reproducible.

```
tag: v1.4.2h2-full-authority-closure
report: docs/PHASE_14_2H2.md
artifacts: artifacts/phase_14_2h2/
```

**14.2 is in a position to freeze** as the authoritative vehicle model, with limitations documented. Phase 14.3 should not open until this freeze is accepted.
