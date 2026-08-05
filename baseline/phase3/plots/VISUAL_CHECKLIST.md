# Combined-Slip Visual Validation Checklist

**Purpose:** Catch non-physical artifacts that numerical tests can miss before integrating the tire model into BrakeSimulation.

**Status:** Required before marking Phase 3.4.2 as "Ready for Integration"

---

## Primary Surface Checks

| Plot | What to look for | Expected | Result |
|------|------------------|----------|--------|
| **Fx surface** | Smooth transition from linear to saturated region | No discontinuities, spikes, or oscillations | ☐ |
| **Fy surface** | Symmetric lateral behavior | Mirror symmetry about α = 0 | ☐ |
| **λ surface** | Monotonic reduction with increasing combined slip | No values outside the physically expected range | ☐ |
| **Utilization** | Gradual approach to the friction limit | No isolated peaks or holes; never exceeds 1.0 | ☐ |
| **Pure longitudinal slice** | Linear near κ = 0, saturating at higher slip | Smooth curve with no kinks | ☐ |
| **Pure lateral slice** | Linear near α = 0, saturating smoothly | Symmetric positive/negative response | ☐ |
| **Combined-slip region** | Increasing longitudinal slip reduces available lateral force (and vice versa) | Smooth trade-off with no non-physical ridges | ☐ |

---

## Red Flags (any of these = STOP and investigate)

- [ ] Sudden jumps or sawtooth patterns in any surface
- [ ] Asymmetry where the tire model should be symmetric (assuming symmetric parameters)
- [ ] Force magnitudes exceeding the imposed friction limit (μ·Fz)
- [ ] Isolated spikes, holes, or numerical noise in the utilization surface
- [ ] λ becoming negative or exploding to very large values

---

## Sign-off

After generating the plots with:

```bash
python -m vehicle_dynamics.tire.visualization
```

Review the images in `baseline/phase3/plots/` and complete the table above.

**Only when all primary checks pass and no red flags are present should the model be marked:**

```
✅ Numerical validation passed
✅ Visual validation passed
✅ Ready for integration into BrakeSimulation
```

Date reviewed: __________  
Reviewer: __________
