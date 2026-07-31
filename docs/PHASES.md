# Development Phases

## Phase 1 – Suspension Dynamics
- 2-DOF quarter-car model
- Skyhook / Groundhook / Hybrid controllers
- ISO 8608 road profiles

## Phase 3 – Braking System
- Longitudinal dynamics + weight transfer + thermal + ABS
- Combined-slip Dugoff (Phase 3.4 – Integrated & Regression Validated)

## Phase 4 – Lateral Dynamics & ESC

### Phase 4.0 – Dynamic Bicycle Model ✅ FROZEN
- 2-DOF (vy, r), combined-slip tires (κ = 0)

### Phase 4.1 – Load Transfer Diagnostics ✅ FROZEN
- Quasi-static left/right load transfer (diagnostics only)

### Phase 4.2 – Combined Braking + Steering (current design)
- Dynamic Vx + vy + r
- Trail braking, combined-slip (κ and α both active)
- Pure-braking and pure-steering regressions

### Phase 4.3 – ESC Foundation
- Desired yaw-rate model, yaw-moment controller, individual brake intervention
