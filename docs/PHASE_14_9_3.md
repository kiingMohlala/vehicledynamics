# PHASE 14.9.3 — Steady-State Cornering & Yaw-Moment Validation

**Status: PASS (22/22 gates)**  
**Date:** 2026-08-15  
**Prerequisite:** 14.9.2 PASS · 14.8 plant frozen  
**No ESC · No retuning**

---

## Mission

```
δ → Ackermann → wheel-local α → Dugoff → ΣFy / ΣMz
  → ay + yaw_acc → steady r
with  ΣFy ≈ m·ay ,  r ≈ ay/vx ,  yaw_acc ≈ 0
```

---

## Critical plant fix

Body-frame integration was missing Coriolis terms. Restored:

```
vẋ = ax + vy · r
vẏ = ay − vx · r
```

Without this, yaw rate ran away while α→0 and ΣFy collapsed — not a physical steady turn.

---

## Steady-state sample (vx≈23 m/s, δ=0.08 rad)

| Metric | Value |
|--------|-------|
| ay | **10.8 m/s²** |
| r | **0.46 rad/s** |
| ΣFy | **11880 N** |
| m·ay | 11901 N (rel err 0.2%) |
| r vs ay/vx | err **0.0%** |
| yaw_acc | ≈0 (balanced Mz) |
| α FL/FR/RL/RR | nonzero, coherent |

---

## Gates (22/22)

Architecture · SS detection · steering/speed authority · L/R symmetry · four-wheel Fy · front/rear split · Ackermann ON/OFF · Fz transfer · μ/Cy · combined slip · yaw-moment balance · force balance · r≈ay/vx · zero-steer · no-wind · crosswind · historical · regression **3.13/8.34** · determinism

---

## Verdict

**PHASE 14.9.3 — PASS**

```
tag: v1.4.9.3-steady-state-cornering
report: docs/PHASE_14_9_3.md
```
