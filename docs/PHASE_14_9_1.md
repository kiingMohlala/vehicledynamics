# PHASE 14.9.1 — Steering Authority & Ackermann Geometry

**Status: PASS (19/19 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** Phase 14.8 plant frozen  
**No retuning** of frozen vehicle identity.

---

## Mission

```
VehicleDefinition → SteeringConfig → SimulationConfig → SteeringModel
  → rate limit → angle limit → Ackermann → δ_FL / δ_FR
  → wheel kinematics → Dugoff α
```

---

## Implementation

| File | Role |
|------|------|
| `steering/steering_config.py` | max_steer_angle, steering_ratio, steering_rate, ackermann_enabled |
| `steering/steering_model.py` | rate/angle limits, classical Ackermann |
| `dual_track_plant.py` | runtime `SteeringModel`; deltas into tire kinematics |

**Ackermann (left turn):**  
`R = L / tan(δ)` → `δ_inner = atan(L/(R−T/2))`, `δ_outer = atan(L/(R+T/2))`  
Rear: δ_RL = δ_RR = 0.

---

## Gates (19/19)

| Gate | Result |
|------|--------|
| Config / runtime authority | PASS |
| Max angle / rate limit | PASS |
| Zero / ± command | PASS |
| Left–right symmetry | PASS |
| Ackermann ON: \|FL\| > \|FR\| | PASS |
| Ackermann OFF: FL = FR | PASS |
| Plant wheel-angle authority | PASS |
| Poisoned defaults resisted | PASS |
| Mutation max / rate | PASS |
| Deterministic ×5 | PASS |
| Historical isolation | 5.37 / 19.81 |
| **Zero-steer 14.8 regression** | **3.13 / 8.34** |
| No identity mutation | PASS |

---

## Verdict

**PHASE 14.9.1 — PASS**

Next: **14.9.2** wheel-local slip angles and steering → tire force coupling (not ESC).

```
tag: v1.4.9.1-steering-ackermann
report: docs/PHASE_14_9_1.md
```
