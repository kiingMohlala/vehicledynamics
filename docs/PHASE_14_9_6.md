# PHASE 14.9.6 — Hydraulic Cross-Linked Anti-Roll Bar

**Status: PASS (17/17 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.5 PASS · 14.8 frozen  
**Passive only — no pump, no ESC, no retuning**

---

## Architecture

```
MechanicalAntiRollBar ──┐
                        ├── AntiRollBar interface → SprungBody → Fz → Dugoff
HydraulicAntiRollBar ───┘
```

`use_hydraulic_arb=True` selects hydraulic; default remains mechanical.

---

## Model

```
δ = z_L − z_R
F_pair = K_h · δ + C_h · δ̇
F_L = −F_pair ,  F_R = +F_pair
ΣF = 0   (load transfer only)
E_orifice ≥ 0
```

Optional derivation: `K_h = 2·β·A²/V` from bulk modulus / geometry.

---

## Evidence

| K_hyd | φ (ay=8) |
|-------|----------|
| 0 | 0.036 |
| 30k | 0.014 |
| 100k | 0.006 |

- Force pair sum = 0  
- Front/rear independent  
- Dissipation E > 0  
- Mechanical fallback intact  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 14.9.6 — PASS**

```
tag: v1.4.9.6-hydraulic-arb
report: docs/PHASE_14_9_6.md
```
