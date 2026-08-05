# Phase 5 Baseline Archive

**Status:** Frozen – Phase 5.0 Dual-Track Architecture: Implementation Validated (Initial)

## Regression Philosophy

The Phase 4.2 bicycle model is the regression reference, **not** the ground truth.
Small steady-state differences (~10–15% in yaw rate) are expected because the
dual-track model resolves wheel-level kinematics and load-transfer effects that
are intentionally lumped in the single-track formulation.

Do not tune dual-track parameters solely to eliminate this difference.

## Freeze Tag

`v0.5.0-phase5.0-dual-track`
