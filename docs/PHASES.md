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
- Selectable tire models (Hard / Simplified Dugoff / Standard Dugoff)
- Combined-slip Dugoff (Phase 3.4 – Integrated & Regression Validated)

## Phase 4 – Lateral Dynamics & ESC

### Phase 4.0 – Dynamic Bicycle Model ✅ FROZEN
- 2-DOF (vy, r)
- Steering input + slip-angle computation
- Combined-slip tire integration (κ = 0)
- Constant-radius and step-steer validation
- Dual ay cross-check, symmetry, linear bicycle comparison

### Phase 4.1 – Load Transfer Coupling (next)
- Lateral load transfer
- Roll moment approximation
- Dynamic normal loads

### Phase 4.2 – Combined Braking + Steering
- Trail braking
- Split-μ corner entry
- Understeer / oversteer studies

### Phase 4.3 – ESC Foundation
- Desired yaw-rate model
- Yaw-moment controller
- Individual wheel brake intervention
