# Phase 5.0 Status

## Phase 5.0 – Dual-Track Architecture: Implementation Validated (Initial) ✅

**Frozen:** 2026-07-31

### Freeze Summary

✅ Four-wheel planar model (FL/FR/RL/RR)  
✅ Independent κ, α, Fz, Fx, Fy, ω, utilization per wheel  
✅ Equal front steering (no Ackermann)  
✅ RK45 integration  
✅ Lateral load-transfer feedback into tire normal loads  
✅ Yaw moments from both Fx and Fy via track geometry  
✅ Tire API unchanged  
✅ ABS reused per-axle  
✅ Symmetric pure-steering / pure-braking regression gates passed  
✅ Load-transfer feedback sign validated  
✅ Steady-state yaw rate within ~12% of Phase 4.2 bicycle (expected)

### Regression Philosophy

> The Phase 4.2 bicycle model is the **regression reference, not the ground truth**.  
> Small steady-state differences are expected because the Phase 5.0 dual-track model  
> resolves wheel-level kinematics and load-transfer effects that are intentionally  
> lumped in the single-track formulation.

Do **not** tune dual-track parameters solely to eliminate the ~10–15% steady-state yaw difference versus the bicycle model.

### Known Limitations (intentional)

- Equal front steering (no Ackermann)
- Per-axle ABS (not per-wheel)
- Lateral load-transfer feedback only
- No longitudinal load transfer
- No suspension compliance or roll dynamics
- No ESC, torque vectoring, or differential model
- Bicycle regression differences of approximately 10–15% in steady-state yaw are expected due to wheel-level geometry and load-transfer fidelity

### Recommended Git Tag
```bash
git tag -a v0.5.0-phase5.0-dual-track \
  -m "Phase 5.0 Dual-Track Architecture: Implementation Validated (Initial)"
git push origin v0.5.0-phase5.0-dual-track
```

### Next

**Phase 5.1 – Ackermann Steering & Independent Wheel Control**
- Ackermann steering geometry
- Per-wheel brake commands
- Demonstrate reduced low-speed scrub
- Re-run all Phase 5.0 regression tests before freeze
