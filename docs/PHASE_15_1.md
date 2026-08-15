# PHASE 15.1 — ESC Observability & Reference Yaw Model

**Status: PASS (14/14 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9 passive dynamics frozen  
**No actuator intervention · No ESC control · No retuning**

---

## Boundary

```
Frozen passive plant
        │
        ├── vx, vy → β
        ├── r, ay, δ
        ▼
  Reference model (observe only)
        │
        ▼
  r_ref, e_r = r − r_ref
        │
        ▼
  ESC decision variables (no commands)
```

**15.1 must not influence the plant.**

---

## Reference model

```
r_kin = (vx / L) · tan(δ)           # neutral bicycle
r_ref = r_kin / (1 + K_us · vx²)    # understeer-corrected
```

| Parameter | Value | Source |
|-----------|-------|--------|
| L | 2.70 m | VehicleDefinition |
| K_us | **0.0065** | 14.9.8 dδ/d(ay) |

Neutral model is **not** used as the ESC reference — that would erase the validated understeer characteristic.

---

## Evidence

| Gate | Result |
|------|--------|
| State observability | β, r, r_ref, e_r finite |
| Zero-steer | r_ref = 0 |
| Sign / reversal | symmetric |
| Understeer vs neutral | \|r_ref\| < \|r_kin\| |
| Low-speed | r_ref → 0 as vx → 0 |
| No actuator authority | ΔΣFy = 0 with/without observer |
| Regression | **3.13 / 8.34 s** |

Example @ 25 m/s, δ≈0.08:  
`r_kin=0.71` · `r_ref=0.15` · measured `r≈0.48` · `e_r≈0.33`

---

## Module

`vehicle_dynamics/controls/esc_observability.py`

- `ESCObservability.observe_from_simulation(sim)` — read-only  
- No `brake_cmd` / `drive_cmd` outputs  

---

## Verdict

**PHASE 15.1 — PASS**

```
tag: v1.5.1-esc-observability-reference-yaw
report: docs/PHASE_15_1.md
```

**Next (15.2):** ESC intervention authority — still no full stability envelope; prove that a *command path* can request differential brake moment without yet closing a control loop on e_r.
