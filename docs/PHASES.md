# Development Phases

## Phase 1 – Suspension Dynamics
- 2-DOF quarter-car model
- Skyhook / Groundhook / Hybrid controllers
- ISO 8608 road profiles
- Full validation suite

## Phase 3 – Braking System
- Longitudinal dynamics + weight transfer
- Wheel dynamics
- Thermal model + fade
- ABS controller (validated)
- Selectable tire models
- Combined-slip Dugoff (Phase 3.4 – Integrated & Regression Validated)

## Phase 4 – Lateral Dynamics & ESC

### Phase 4.0 – Dynamic Bicycle Model ✅ FROZEN
- 2-DOF (vy, r)
- Steering + slip angles
- Combined-slip tires (κ = 0)
- Full validation suite

### Phase 4.1 – Load Transfer Coupling (current design)
- Quasi-static lateral load transfer
- Dynamic / diagnostic normal loads
- Zero-ay regression against Phase 4.0

### Phase 4.2 – Combined Braking + Steering
- Trail braking
- Split-μ corner entry
- Understeer / oversteer studies

### Phase 4.3 – ESC Foundation
- Desired yaw-rate model
- Yaw-moment controller
- Individual wheel brake intervention
