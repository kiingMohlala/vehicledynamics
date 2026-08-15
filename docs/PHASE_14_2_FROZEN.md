# PHASE 14.2 — AUTHORITATIVE VEHICLE MODEL FROZEN

**Status: FROZEN — PASS**  
**Date:** 2026-08-15  
**Final authority tag:** `v1.4.2-frozen`  
**Preceding closure:** `v1.4.2h2-full-authority-closure` (commit e2cedb8)

---

## Authoritative vehicle

| Parameter | Value |
|-----------|-------|
| Mass | **1100 kg** |
| Peak power | **750 kW** |
| Drive | **AWD** (split 0.35 front) |
| Tire μ | **1.15** |
| Wheel radius | **0.33 m** |
| Final drive | **3.9** |
| Gears | **[3.5, 2.2, 1.6, 1.2, 1.0, 0.85]** |
| Plant | dual-track + Dugoff + ABS + aero |

---

## Authority chain

```
VehicleDefinition
      ↓
SimulationConfig
      ↓
Runtime plant
      ↓
Telemetry / validation
```

Proven at runtime (14.2H.1 / 14.2H.2): mass, power, torque, μ, Cx, Cy, radius, FD, gear vector, AWD split, brake Tmax, ABS, aero coefficients, geometry (wheelbase, tracks, h_cg). Mutation tests and poisoned-default tests confirm no silent library override on the authoritative path.

---

## Historical demonstrator

| Parameter | Value |
|-----------|-------|
| Mass / power | **1400 kg / 280 kW** |
| Role | **Regression only** |
| 0–100 / 0–200 | **5.36 s / 19.77 s** |

Isolated via `historical_demonstrator_config()` / `bind_historical_demonstrator()`. Must not be mutated by hypercar execution.

---

## Hypercar reference performance (frozen evidence)

| Metric | Value |
|--------|-------|
| 0–100 km/h | **3.13 s** |
| 0–200 km/h | **8.31 s** |
| Deterministic replay (n=5) | identical |

These numbers are model predictions, not targets. They were not achieved by retuning.

---

## Known limitations (accepted)

1. **Energy: PARTIAL** — residual_frac ≈ 0.5%; small residual ≠ fully explained residual. Transmission internal loss remains a residual proxy.
2. **Crosswind** — external disturbance model, not aero β-sideforce.
3. **Load transfer** — quasi-static (no suspension travel ODE).
4. **Manoeuvres** — open-loop; no closed-loop driver model in the frozen plant path.

None of these limitations indicates a broken authority chain.

---

## Freeze policy

- **Do not reopen 14.2 architecture** unless a regression failure exposes an actual defect in the authority chain or plant coupling.
- Do not retune mass, power, μ, tire stiffness, aero, gearing, or controller gains to chase performance numbers.
- Phase 14.3 (if opened) must treat this vehicle as the frozen reference, not redefine it.

---

## Verdict

**PHASE 14.2 — FROZEN — PASS**
