# PHASE 14.9.9 — Combined Braking + Cornering & Combined-Slip Validation

**Status: PASS (20/20 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.8 PASS · 14.8 frozen  
**No ESC · No ABS retuning · No vehicle-identity changes**

---

## Mission

```
δ → α          ↘
                Dugoff(κ, α, Fz) → Fx, Fy → ΣFx/ΣFy/ΣMz → ax/ay/yaw
brake → κ      ↗
```

Prove the friction budget is shared correctly:

```
β = √[(Fx/μFz)² + (Fy/μFz)²]  ≤ ~1
```

---

## Friction budget (fixed δ=0.10, increasing brake)

| Brake | |Fy| | β_max |
|-------|------|--------|
| 0.0 | 13064 | 0.97 |
| 0.2 | 11509 | 0.99 |
| 0.4 | 10223 | 0.99 |
| 0.6 | 9990 | 0.99 |

Heavy braking reduces lateral capacity as Fx consumes the ellipse.

---

## Other results

- Pure brake: ax ≈ −9.5  
- Pure corner: ay ≈ 11.3  
- Light combined: ΣFx −4944, ΣFy 11746  
- μ×0.5 cuts both Fx and Fy  
- L/R symmetry · recovery r 0.83→0.00  
- Regression **3.13 / 8.34 s**

---

## Verdict

**PHASE 14.9.9 — PASS**

```
tag: v1.4.9.9-combined-slip-brake-corner
report: docs/PHASE_14_9_9.md
```

---

## Passive vehicle dynamics closure

```
14.8  Core plant frozen
14.9.1 Steering / Ackermann
14.9.2 Wheel-local α
14.9.3 Steady cornering
14.9.4 Transient yaw
14.9.5 Mechanical ARB
14.9.6 Hydraulic ARB
14.9.7 Roll-stiffness distribution
14.9.8 Understeer / oversteer
14.9.9 Combined-slip brake + corner   ← PASSIVE LATERAL + LONGITUDINAL CLOSED
        ↓
   15.x CONTROL SYSTEMS (ESC / active)
```
