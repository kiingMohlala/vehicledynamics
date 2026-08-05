# Phase 5.1 Status

## Phase 5.1 – Ackermann Steering & Independent Front Wheel Steering: Implementation Validated ✅

**Frozen:** 2026-08-03

### Freeze Summary

✅ `dual_track/steering.py` – Ackermann geometry + `SteeringParameters`  
✅ Independent `delta_fl` / `delta_fr` (rear remain 0)  
✅ Per-wheel slip-angle and force transforms use wheel-specific steer  
✅ `use_ackermann=False` recovers Phase 5.0 equal-steer behaviour  
✅ Tire API, braking, load-transfer feedback unchanged  
✅ Full geometry + simulation validation passed  
✅ Phase 5.0 regression suite passes unchanged with equal steer

### Validation Results (2026-08-03)

| Gate | Result |
|------|--------|
| zero_steer | PASS |
| left_right_symmetry | PASS |
| inside_outside | PASS |
| low_speed_geometry (cot residual) | PASS |
| Phase 5.0 pure steering (equal) | PASS |
| Phase 5.0 pure braking | PASS |
| Phase 5.0 load-transfer feedback | PASS |
| Phase 5.0 no NaN / util ≤ 1 | PASS |
| Ackermann simulation smoke | PASS |
| Parallel vs Ackermann comparison | PASS |

### Parallel vs Ackermann (5° step, 15 m/s)

| Metric | Parallel | Ackermann |
|--------|----------|-----------|
| r_ss [rad/s] | 0.448 | 0.441 |
| path radius [m] | 33.9 | 34.5 |
| δ_fl [deg] | 5.00 | 5.13 |
| δ_fr [deg] | 5.00 | 4.88 |
| max utilization | 0.883 | 0.876 |

### Recommended Git Tag

```bash
git tag -a v0.5.1-phase5.1-ackermann \
  -m "Phase 5.1 Ackermann Steering & Independent Front Wheel Steering: Implementation Validated"
git push origin v0.5.1-phase5.1-ackermann
```

### Known Scope Limits

- Per-axle ABS (not per-wheel pressure)
- No ESC / torque vectoring
- No longitudinal load-transfer feedback
- No dynamic roll DOF

### Next

Phase 5.2 – ESC foundation / brake vectoring, or per-wheel brake modulation.
