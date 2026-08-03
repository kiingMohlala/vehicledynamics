# Version 1.0 – Engineering Baseline

**Tag:** `v1.0-engineering-baseline`  
**Date:** 2026-08-03

This is the permanent reference for the validated vehicle dynamics and controls platform **before** suspension kinematics.

## Contents

| Domain | Capability |
|--------|------------|
| Suspension (ride) | 2-DOF quarter-car, Skyhook/Groundhook/Hybrid, ISO 8608 roads |
| Semi-active | MR damper interface (Phase 2) |
| Tires | Combined-slip Dugoff (longitudinal + lateral) |
| Braking | Longitudinal dynamics, thermal, fade, energy/passivity |
| ABS | Per-wheel FSM |
| Lateral | Bicycle model, dual-track 4-wheel |
| Steering | Ackermann |
| Load transfer | Lateral diagnostics + feedback |
| Stability | ESC |
| Drive | Active torque vectoring |
| Quality | Phase 5.5 system benchmark (12/12) |

## Tag

```bash
git tag -a v1.0-engineering-baseline \
  -m "First complete validated vehicle dynamics and controls platform"
git push origin v1.0-engineering-baseline
```

## Policy

Every future physics change (suspension geometry, aero, chassis flexibility, powertrain) must be regression-tested against this baseline and the Phase 5.5 benchmark report.
