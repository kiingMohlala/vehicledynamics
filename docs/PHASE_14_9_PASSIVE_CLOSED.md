# PASSIVE VEHICLE DYNAMICS — CLOSED

**Date:** 2026-08-15  
**Foundation tag:** `v1.4.8-coupled-plant-validated`  
**Closure tag:** `v1.4.9.9-combined-slip-brake-corner`

## Stack

| Phase | Topic |
|-------|--------|
| 14.8 | Core plant frozen |
| 14.9.1 | Steering / Ackermann |
| 14.9.2 | Wheel-local slip angles |
| 14.9.3 | Steady-state cornering |
| 14.9.4 | Transient lateral / yaw |
| 14.9.5 | Mechanical ARB |
| 14.9.6 | Hydraulic ARB |
| 14.9.7 | Roll-stiffness distribution |
| 14.9.8 | Understeer / oversteer characterization |
| 14.9.9 | Combined-slip braking + cornering |

## Frozen longitudinal reference

| Vehicle | 0–100 | 0–200 |
|---------|-------|-------|
| Hypercar | 3.13 s | 8.34 s |
| Historical | 5.37 s | 19.81 s |

## Handling summary

Natural balance: **understeer** (dδ/d(ay) > 0)  
Yaw-stable to tire limit · friction ellipse respected under combined slip

## Policy

Do not reopen 14.8–14.9 for retuning.  
Control systems (ESC, active roll, TV) start at **15.x** on this substrate.
