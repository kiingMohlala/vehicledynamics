# PHASE 14.7 — Unsprung Mass, Wheel-Hop & Tire-Load Dynamics

**Status: PASS (27/27 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.6 PASS  
**No retuning** of frozen mass / power / μ / gear identity.

---

## Mission

```
Road
  ↓
Tire (k_t, c_t)
  ↕
Unsprung mass (m_u)
  ↕
Suspension (k, c)
  ↕
Sprung body (z, θ, φ)
  ↓
Dynamic Fz → Dugoff
```

---

## Implementation

**`vehicle_dynamics/simulation/unsprung_model.py`**

- Four independent `z_u`, `ż_u` states  
- `F_tire_on_u = −k_t(z_u − z_road) − c_t(ż_u − ż_road)`  
- `F_susp_on_u = +k(δ) + c(δ̇)` with `δ = z_s − z_u` (Newton 3)  
- Contact: `Fz = Fz_static + F_tire_on_u` (floor `Fz_min`)  

**Orchestration in `DualTrackPlant`:** corner positions → unsprung step → sprung step with `Fz_contact`.

**Authority:** `m_u`, `k_tire`, `c_tire` on `SimulationConfig` → `DualTrackConfig` → `UnsprungModel`.

---

## Gates (27/27)

| Gate | Result |
|------|--------|
| Architecture / parameter authority | PASS |
| Static equilibrium / vertical balance | ΣFz ≈ mg |
| Single-wheel bump | FzFL 3875 → **12635** |
| Road isolation | ΔFzFL ≫ ΔFzRR |
| Four-wheel road | transient peak then settle ≈ mg |
| Wheel-hop frequency | **~17 Hz** (m_u=40, k_t=220k) |
| m_u ×2 → T ↑ (0.058 → 0.084 s) | PASS |
| k_tire ×2 → T ↓ | PASS |
| Tire / susp damper E ≥ 0 | PASS |
| Dynamic Fz ≠ static path | PASS |
| Dugoff coupling | PASS |
| Brake / corner / combined | PASS |
| Deterministic ×5 | PASS |
| Historical isolation | 5.37 / 19.81 |
| Regression (reported, not retuned) | hyper 3.13 / 8.34 |
| Poisoned defaults resisted | PASS |

---

## Regression vs 14.6

| Vehicle | 14.6 | 14.7 | Δ |
|---------|------|------|---|
| Hypercar 0–100 | 3.16 s | **3.13 s** | −0.03 |
| Hypercar 0–200 | 8.39 s | **8.34 s** | −0.05 |
| Historical 0–100 | 5.37 s | **5.37 s** | 0 |
| Historical 0–200 | 19.81 s | **19.81 s** | 0 |

---

## Limitations

1. Linear tire vertical spring (no progressive / loss of contact model beyond Fz_min).  
2. No lateral/longitudinal tire compliance.  
3. Road input is kinematic `road_z` per wheel (no terrain mesh).  
4. Global drivetrain energy still PARTIAL.

---

## Verdict

**PHASE 14.7 — PASS**

```
tag: v1.4.7-unsprung-wheel-hop
report: docs/PHASE_14_7.md
```
