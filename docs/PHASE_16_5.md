# PHASE 16.5 — ESC Candidate Freeze & Release Validation

**Status: PASS (18/18 gates)**  
**Date:** 2026-08-19  

---

## Freeze

```
K_Mz = 10000        🔒 FROZEN ESC CALIBRATION
ESC architecture    🔒 FROZEN
14.9 plant          🔒 FROZEN
K_us = 0.0065       🔒 FROZEN (plant characterization)
```

Changing `K_Mz` requires a **new controlled calibration/validation phase**.

---

## Final configuration

| Parameter | Value |
|-----------|-------|
| K_Mz | 10000 |
| K_us | 0.0065 |
| e_enter / e_exit | 0.12 / 0.06 |
| max_delta_Mz | 6000 |
| max_util / max_beta | 0.98 / 0.45 |
| min_vx / min_delta | 8.0 / 0.015 |
| brake_torque_max | 2800 |
| wheelbase / track_f / track_r | 2.70 / 1.65 / 1.62 |

---

## Validation chain

15.5 → 15.9 · 16.1 → 16.4 · **all PASS** · final subset regression PASS · ESC-OFF **3.13 / 8.34 s**

---

## Artifacts

- `artifacts/phase_16_5/final_esc_config.json`
- `artifacts/phase_16_5/final_regression.json`
- `artifacts/phase_16_5/freeze_manifest.json`

```
tag: v1.6.5-esc-candidate-frozen
```
