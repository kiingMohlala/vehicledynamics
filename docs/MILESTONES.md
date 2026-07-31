# Project Milestones

## v0.5.0-phase5.0-dual-track (2026-07-31)

**Status:** Frozen – Implementation Validated (Initial)

### Phase 5.0 – Dual-Track Architecture: Implementation Validated (Initial)

✅ FL/FR/RL/RR independent wheels  
✅ Lateral load-transfer feedback  
✅ Symmetric regression vs Phase 4.2 bicycle (expected ~10–15% steady yaw difference)  
✅ No Ackermann / no ESC yet

### Regression Philosophy
Phase 4.2 bicycle is the regression reference, not ground truth. Wheel-level kinematics and load-transfer fidelity intentionally produce small steady-state differences.

### Recommended Git Tag
```bash
git tag -a v0.5.0-phase5.0-dual-track \
  -m "Phase 5.0 Dual-Track Architecture: Implementation Validated (Initial)"
git push origin v0.5.0-phase5.0-dual-track
```

---

## Previous Freezes

| Tag | Description |
|-----|-------------|
| v0.4.2-phase4.2-combined | Combined Braking & Steering: Implementation Validated |
| v0.4.1-phase4.1-load-transfer | Load Transfer Diagnostics: Implementation Validated |
| v0.4.0-phase4-frozen | Dynamic Bicycle Model: Implementation Validated |
| v0.3.4-phase3.4-combined-slip | Combined-Slip Dugoff: Integrated & Regression Validated |
| v0.3.3-phase3-frozen | Braking + ABS + Standard Dugoff |
