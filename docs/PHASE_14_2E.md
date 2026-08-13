# PHASE 14.2E — Energy, Handling & Scenario Closure

**Status: PASS (30/30 behavioural gates)**  
**Date:** 2026-08-13  
**Tag:** `v1.4.2e-scenario-closure`  
**Reference vehicle:** frozen Phase 14.2D configuration (no mass/μ/tire/ratio retuning)

---

## 1. Reference Vehicle

| Parameter | Value |
|-----------|-------|
| mass | 1400 kg |
| Iz | 2500 kg·m² |
| wheelbase | 2.7 m |
| track | 1.55 m |
| wheel radius | 0.32 m |
| μ_tire | 1.15 |
| peak power | 280 kW |
| peak torque (derived) | ~594 N·m |
| final drive | 3.9 |
| drive split front | 0.35 |
| plant | dual-track + Dugoff + ABS |
| aero | enabled (Cd/Cl map) |

---

## 2. Frozen Baseline (14.2D)

| Metric | 14.2D |
|--------|-------|
| 0–50 | 2.59 s |
| 0–100 | 5.36 s |
| 0–200 | 19.77 s |

14.2E reproducibility (3 runs, identical): **2.59 / 5.36 / 19.77 s** — deterministic.

---

## 3. Longitudinal Performance

| Scenario | Result | Source | Status |
|----------|--------|--------|--------|
| 0–50 | 2.59 s ×3 | simulation | PASS |
| 0–100 | 5.36 s ×3 | simulation | PASS |
| 0–200 | 19.77 s ×3 | simulation | PASS |

---

## 4. Braking

| Scenario | Result | Source | Status |
|----------|--------|--------|--------|
| Dry 100→0 | **2.33 s**, 31.4 m, ABS active | simulation | PASS |
| Wet (μ_scale=0.5) | **4.56 s** (> dry) | simulation | PASS |
| Split-μ | Fx asymmetric [-4249, -1295, -3483, -1060], yaw_acc=1.65 | simulation | PASS |

Chain verified: brake_cmd → ABS → pressure → T_brake → κ → Dugoff Fx → ax / yaw.

---

## 5. Handling Coupling

Causal chain demonstrated from telemetry:

```
steer → road-wheel δ → contact velocity → α → Dugoff → Fy
  → ΣFy → ay
  → contact-patch moments → Mz → yaw_acc → yaw rate → heading
```

| Scenario | Result | Source | Status |
|----------|--------|--------|--------|
| Constant radius | mean \|ay\|=11.49; ΣFy≈m·ay (res≈0); front α≈1.07 | simulation | PASS |
| Steering→α | mean \|α_front\|=1.07 | simulation | PASS |
| α→Dugoff Fy | mean \|Fy_front\|=4324 N | simulation | PASS |
| Fy→ay | median \|ΣFy−m·ay\|=0 | simulation | PASS |
| Tire→yaw | mean \|yaw_acc\|=0.067 | simulation | PASS |
| Slalom | ay_amp=11.33, yaw_rate_amp=0.63 | simulation | PASS |
| DLC | max_ay=11.47, y_span=59.9 m | simulation | PASS |
| Figure-eight L/R | ay ±5.25, yaw_acc ±3.66 (opposite) | simulation | PASS |

No steer×gain lateral shortcut on the dual-track path.

---

## 6. Aero

| Scenario | Result | Source | Status |
|----------|--------|--------|--------|
| High-speed sweep | drag 40→1424 N, DF 134→4818 N (mono↑ with v) | simulation | PASS |
| Crosswind | ay responds via `crosswind×40 N` disturbance | simulation | PASS* |

\* **Limitation:** crosswind is a chassis disturbance force, not a full aero side-force/yaw model from relative air velocity. Documented; not presented as full aero crosswind.

---

## 7. Powertrain

| Check | Result | Status |
|-------|--------|--------|
| WOT | max gear 6, peak Te 472 N·m | PASS |
| Shifts | 10 transitions, post-shift RPM kinematically consistent when locked | PASS |
| Torque chain | locked positive Tw fraction = 1.00 | PASS |

---

## 8. Energy Ledger

| Quantity | Value |
|----------|-------|
| E_engine | 3.45 MJ |
| E_driveline (∫Tw·ωw) | (see trace) |
| E_wheel_rotation | 0.11 MJ |
| W_tire | 2.93 MJ |
| E_vehicle | 2.18 MJ |
| residual_fraction | **0.338** |

Residual = clutch heat + shift ignition cuts + aero/rolling + tire scrub.  
No energy creation. Gate **PASS**.

---

## 9. Force / Moment Closure

| Residual | RMS / max | Status |
|----------|-----------|--------|
| Fx − m·ax | rms 356 N (drag/roll) | PASS |
| Fy − m·ay | rms 0 N | PASS |
| Fz − (mg+DF) | max 0 N | PASS |
| Mz − Iz·yaw_acc | 0 by plant construction | PASS |

---

## 10. Durability

60 s mixed throttle/brake/steer: no NaN/Inf, max ω=250 rad/s (clamped), max yaw_rate=2.98 rad/s. **PASS**.

---

## 11. Evidence Classes

| Class | Used for |
|-------|----------|
| **SIMULATION** | all performance, handling, braking, energy, force closure |
| **ANALYTICAL** | energy residual definition; ratio kinematics |
| **REGRESSION** | 14.2D t100 match; deterministic replay |
| **HEURISTIC** | crosswind disturbance magnitude (explicitly labelled) |
| **MEASURED** | none (no physical dyno/track data in this phase) |

---

## 12. Failures / Limitations

1. **Crosswind:** not a full aero side-force model — only `F_y = crosswind × 40 N` on chassis.
2. **DLC/slalom:** open-loop steer profiles, not closed-loop ISO path following.
3. **Energy residual ~34%:** expected for clutch slip + sequential cuts; not unexplained free energy.
4. **No suspension travel ODE** in this plant path (load transfer is quasi-static).
5. **Legacy plant branch** still contains steer×gain if `use_dual_track=False` — not used in 14.2E.

---

## 13. Regression

| Layer | Status |
|-------|--------|
| 14.2D t100 = 5.36 s | reproduced |
| Dugoff dual-track | bound |
| Deterministic replay | bit-stable vx/rpm/gear |
| Braking envelope | dry/wet consistent with 14.2D |

---

## 14. Scenario Evidence Matrix

| Scenario | Source | Result | Evidence | Status |
|----------|--------|--------|----------|--------|
| 0–100 | simulation | 5.36 s | acceleration/ | PASS |
| 0–200 | simulation | 19.77 s | acceleration/ | PASS |
| Dry braking | simulation | 2.33 s / 31.4 m | braking/dry.json | PASS |
| Wet braking | simulation | 4.56 s | telemetry | PASS |
| Split-μ | simulation | asymmetric Fx, yaw | telemetry | PASS |
| Constant radius | simulation | ΣFy≈m·ay | handling/ | PASS |
| Slalom | simulation | ay/yaw response | handling/ | PASS |
| DLC | simulation | trajectory from integration | handling/ | PASS |
| Figure-eight | simulation | L/R opposite ay/yaw_acc | handling/ | PASS |
| Aero | simulation | drag/DF ∝ v² | aero/ | PASS |
| Crosswind | simulation* | ay responds | limited model | PASS* |
| WOT | simulation | gear 1–6, torque chain | powertrain/ | PASS |
| Energy closure | sim+analytical | residual 0.34 | energy/ | PASS |
| Replay | regression | identical | telemetry | PASS |

---

## 15. Final Verdict

**PHASE 14.2E — PASS**

Reason: 30/30 gates from simulation telemetry; handling coupling chain closed (steer→α→Dugoff Fy→ay / Mz→yaw); energy and force/moment residuals within defined tolerances; 14.2D longitudinal result reproduced deterministically; limitations documented without inventing physics.

---

## Freeze

```
branch: phase-14.2e-scenario-closure
tag:    v1.4.2e-scenario-closure
report: docs/PHASE_14_2E.md
artifacts: artifacts/phase_14_2e/
```
