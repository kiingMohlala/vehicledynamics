# Phase 5.5 Benchmark Report

**Status:** ALL PASSED
**Date:** 2026-08-03
**Scenarios:** 16/16 passed

| Scenario | Controllers | Peak |r| | Peak ay | Max util | ABS | ESC% | TV% | OK |
|----------|-------------|----------|---------|----------|-----|------|-----|----|
| constant_radius | open | 0.363 | 5.52 | 0.829 | 0 | 0.00 | 0.00 | Y |
| step_steer | open | 0.652 | 9.38 | 0.992 | 0 | 0.00 | 0.00 | Y |
| sine_steer | open | 0.452 | 6.73 | 0.895 | 0 | 0.00 | 0.00 | Y |
| double_lane_change | open | 0.553 | 8.65 | 0.979 | 0 | 0.00 | 0.00 | Y |
| emergency_brake | ABS | 0.000 | 0.00 | 0.980 | 130 | 0.00 | 0.00 | Y |
| emergency_brake_no_abs | open | 0.000 | 0.00 | 0.981 | 0 | 0.00 | 0.00 | Y |
| split_mu_brake | ABS | 0.057 | 0.83 | 0.981 | 119 | 0.00 | 0.00 | Y |
| trail_braking | ABS | 0.464 | 7.13 | 0.966 | 1 | 0.00 | 0.00 | Y |
| straight_accel | open | 0.000 | 0.00 | 0.981 | 0 | 0.00 | 0.00 | Y |
| corner_exit | TV | 2.617 | 9.42 | 1.000 | 0 | 0.00 | 0.59 | Y |
| split_mu_launch | TV | 0.052 | 0.69 | 0.993 | 0 | 0.00 | 0.29 | Y |
| brake_while_steer | ABS+ESC | 0.674 | 8.21 | 0.988 | 16 | 0.35 | 0.00 | Y |
| power_on_oversteer | ESC+TV | 2.726 | 9.42 | 1.000 | 0 | 0.72 | 0.53 | Y |
| lift_off_oversteer | ESC | 1.451 | 9.64 | 1.000 | 0 | 0.77 | 0.00 | Y |
| esc_tv_combined | ABS+ESC+TV | 2.540 | 9.44 | 1.000 | 6 | 0.71 | 0.69 | Y |
| abs_esc_tv_all | ABS+ESC+TV | 1.878 | 9.61 | 1.000 | 22 | 0.72 | 0.73 | Y |

## Gold-standard baseline
This is the Phase 5 system regression baseline.
Future physics changes must re-run and compare against this report.