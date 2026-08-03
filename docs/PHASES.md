# Development Phases

## Phase 1 – Suspension Dynamics
## Phase 3 – Braking System ✅
## Phase 4 – Lateral Dynamics ✅
- 4.0 Bicycle · 4.1 Load-transfer diagnostics · 4.2 Combined braking & steering

## Phase 5 – Dual-Track & Control

### Phase 5.0 – Dual-Track Architecture ✅ FROZEN (Initial)
- FL/FR/RL/RR independent wheels
- Lateral load-transfer feedback
- Symmetric regression vs bicycle (expected ~10–15% steady yaw difference)

### Phase 5.1 – Ackermann Steering & Independent Front Angles (current)
- `steering.py` Ackermann geometry
- Independent `delta_fl` / `delta_fr`
- `use_ackermann=False` recovers Phase 5.0 equal-steer
- Validation: zero steer, symmetry, inside/outside, geometry residual, Phase 5.0 compat

### Phase 5.2+ (later)
- ESC / brake vectoring
- Differentials and torque distribution
- Optional dynamic roll
