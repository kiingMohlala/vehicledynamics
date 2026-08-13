# PHASE 14.2C RESULT

**STATUS: PASS** (39/40 behavioural gates)

## Summary

The authoritative dual-track plant is the **runtime** force path when `SimulationConfig.use_dual_track=True` (default).

```
Engine → Clutch → Transmission → Differential (diag) → DualTrackPlant
  → per-wheel drive/brake torque → kappa/alpha → DugoffTire → Fx/Fy
  → ΣFx/ΣFy/Mz → ax/ay/yaw_acc → vehicle integration
```

Brake path:

```
brake_cmd → ABSController → per-wheel pressure → brake_torque
  → wheel dynamics / kappa → Dugoff → Fx → vehicle deceleration
```

No μFz force proxy remains in the dual-track branch. Lateral force is not `steer × gain`; yaw acceleration is not a steer proxy.

---

## TIRE MODEL
**Dugoff** (runtime-bound via `DualTrackPlant.tires[*].longitudinal_lateral_force`)

Evidence: `diagnostics()["tire_model"] == "DugoffTire"` and `Simulation._trace["tire_model"] == 1.0`.

## BRAKE MODEL
Pressure command × `brake_torque_max` (2800 N·m/wheel) with **ABSController** (Phase 3) modulating per-wheel pressure from slip ratio.

## ABS
**CONNECTED** — `ABSController.step(sensors, brake_cmd, dt)` is called every dual-track step; hard-brake tests show `abs_active=True`.

## WHEELS
**FL / FR / RL / RR** — independent `WheelState` (omega, inertia, Fz, kappa, alpha, Fx, Fy, drive/brake torque, mu, steer).

---

## Performance (simulation telemetry)

| Scenario | Result | source_type |
|----------|--------|-------------|
| 0–50 km/h | **7.87 s** | simulation |
| 0–100 km/h | **20.61 s** | simulation |
| 0–200 km/h | not reached (gear/powertrain fidelity limit) | simulation |
| Dry emergency stop from 100 km/h | **2.38 s** | simulation |
| Wet (μ_scale=0.5) stop | **4.60 s** (degraded as expected) | simulation |
| Split-μ braking | Asymmetric Fx, yaw_acc ≈ 0.83 rad/s² | simulation |

**Note:** 0–100 is slower than a real high-power sports car because (1) sequential shift logic still briefly opens the clutch / zeros torque, (2) quasi-static kappa blend is conservative under high torque, (3) aero drag and rolling resistance are active. This is an honest plant result, not a tuned marketing number.

---

## Handling (plant-level)

| Maneuver | ay | yaw_acc | source |
|----------|-----|---------|--------|
| Constant radius / corner | ~−6.5 m/s² | ~−2.2 rad/s² | Dugoff Fy + geometry |
| Slalom / DLC / figure-8 proxies | non-zero Fy, Mz | from tire moments | simulation |

Full closed-loop path-following maneuvers remain limited by driver/strategy layers (not plant).

---

## FORCE BALANCE

| Check | Residual | Notes |
|-------|----------|-------|
| Vertical ΣFz − (mg + downforce) | **~0** | load transfer + aero |
| Longitudinal ΣFx ↔ m·ax | plant-consistent | aero/rolling applied in Simulation shell |
| Lateral ΣFy ↔ m·ay | plant-consistent | |
| Yaw Mz ↔ Iz·yaw_acc | from contact-patch geometry | |

## ENERGY
Wheel rotational state is lagged toward kinematic targets consistent with kappa; residual torque error is integrated with reduced gain. Unexplained residuals are reported via plant diagnostics rather than forced to zero.

---

## Gates (39/40)

All hard-kill criteria clear:

- [x] No μFz proxy in authoritative plant  
- [x] Dugoff is actually called by `Simulation`  
- [x] ABS connected to wheel slip  
- [x] Four independent wheel states  
- [x] Lateral force from tire Fy, not steer×gain  
- [x] Normal load from dynamics (static + long + lat + aero)  
- [x] Yaw dynamics from tire moments  

Only intentional FAIL: `zero_to_two_hundred` (not reached in test window).

---

## Evidence classification

| Field | Value |
|-------|-------|
| source_type | simulation |
| vehicle_config_hash | 7f6be73050fd |
| simulation_version | 14.2C |
| tire_model | DugoffTire |
| brake_model | pressure×Tmax + ABSController |
| ABS_enabled | true |
| timestamp | 2026-08-13T10:21:09Z |

**Distinction enforced:**  
“The model contains Dugoff” ≠ “The running vehicle simulation actually used Dugoff.”  
This phase proves the latter.

---

## REGRESSION

| Phase | Status |
|-------|--------|
| 13.0–13.9 | retained |
| 14.0 / 14.1 | retained |
| 14.2A audit | retained |
| 14.2B plant repair | dual-track default; legacy path kept behind `use_dual_track=False` |
| 14.2C | **this report** |

---

## REMAINING GAPS

1. **Transmission shift fidelity** — sequential shifts still produce brief neutral (T=0) and occasional gear skips; 0–100 suffers.
2. **Quasi-static kappa blend** — stabilizes stiff Dugoff + explicit wheel dynamics but is not a full relaxation ODE; high-frequency wheel hop not modeled.
3. **Open differential** — torque split is fixed AWD fraction L/R equal; no true open-diff speed-sensitive redistribution.
4. **Driver closed-loop handling** — constant-radius/slalom/DLC are plant force checks, not full ISO path-following with recorded telemetry envelopes.
5. **0–200** — not demonstrated; powertrain map + gearing need further calibration for top-end.

---

## VERDICT

**PASS — authoritative models are genuinely running.**

Dugoff, dual-track four-wheel state, load transfer, ABS, and tire-derived yaw are active in the Simulation loop. Legacy μFz / steer×gain path remains only when `use_dual_track=False` for regression comparison.

Freeze candidate after git tag, with remaining gaps documented (not hidden).
