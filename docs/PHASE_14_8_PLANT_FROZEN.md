# CORE VEHICLE PLANT — FROZEN

**Tag:** `v1.4.8-coupled-plant-validated`  
**Date:** 2026-08-15  
**Status:** FROZEN — PASS

## Authoritative vehicle

1100 kg · 750 kW · AWD · μ=1.15 · r=0.33 m · FD=3.9  
gears=[3.5, 2.2, 1.6, 1.2, 1.0, 0.85]

## Plant stack

```
VehicleDefinition
      ↓
SimulationConfig
      ↓
Runtime Plant
  • Relative-airflow aero (Cyβ, Cnβ)
  • Sprung body (z, θ, φ)
  • Suspension ↔ Unsprung (4×)
  • Tire vertical → contact Fz
  • Dugoff Fx/Fy + ABS
  • Powertrain torque chain
```

## Reference performance

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | 3.13 s | 8.34 s |
| Historical 1400/280 | 5.37 s | 19.81 s |

## Policy

Do not reopen 14.2–14.8 architecture unless regression exposes a defect.  
Next capability (14.9+) must treat this plant as frozen substrate.
