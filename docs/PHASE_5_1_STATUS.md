# Phase 5.1 Status

## Phase 5.1 – Ackermann Steering & Independent Front Wheel Angles

**Status:** Implementation in progress (validation required before freeze)

### Delivered

- `dual_track/steering.py` – Ackermann geometry + `SteeringParameters`
- Independent `delta_fl` / `delta_fr` (rear remain 0)
- Slip-angle / force transforms use per-wheel steer angles
- `use_ackermann=False` recovers Phase 5.0 equal-steer behaviour
- Tire API, braking, load-transfer feedback unchanged

### Validation suite – `validation_steering.py`

| Test | Intent |
|------|--------|
| zero_steer | δ=0 → both front angles 0 |
| left_right_symmetry | −δ mirrors +δ |
| inside_outside | |δ_inside| > |δ_outside| |
| low_speed_geometry | cot residual ≈ 0 |
| phase50_equal_steer_compat | equal-steer mode matches Phase 5.0 |
| ackermann_simulation_smoke | dynamics finite with Ackermann on |

### How to run

```bash
python -m vehicle_dynamics.dual_track.validation_steering
```

### Not yet in this milestone

- Per-wheel ABS / independent brake pressure modulation beyond equal axle split
- Dynamic roll, longitudinal load-transfer feedback, ESC
