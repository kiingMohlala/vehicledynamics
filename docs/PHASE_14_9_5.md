# PHASE 14.9.5 — Anti-Roll Bar Dynamics & Roll-Stiffness Authority

**Status: PASS (26/26 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.4 PASS · 14.8 frozen  
**No ESC · No retuning**

---

## Mission

```
ay → φ → Δz = z_L − z_R → ARB → equal/opposite F → Fz redistribute → Dugoff
```

---

## Implementation

**`vehicle_dynamics/suspension/anti_roll_bar.py`**

- `MechanicalAntiRollBar`: `F_pair = K·Δz + C·Δż`, equal/opposite forces  
- `DualAxleARB`: independent front/rear  
- Net vertical load per axle: **0**  
- Wired into `SprungBodyModel` (replaces pure φ-proxy when `use_arb=True`)

---

## Evidence

| Config | φ under ay=8 |
|--------|----------------|
| K=0 | 0.036 |
| K=25k/22k | 0.016 |
| K=100k/90k | 0.006 |
| Front-only 100k | 0.010 |
| Rear-only 100k | 0.010 |

- Force pair sum = 0  
- ΣFz = mg conserved  
- L/R symmetry · reversal · damping authority  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 14.9.5 — PASS**

```
tag: v1.4.9.5-anti-roll-bar
report: docs/PHASE_14_9_5.md
```

Next (optional): 14.9.6 hydraulic cross-linked ARB comparison — not ESC.
