# Phase 3 Baseline Archive

**Status:** Frozen reference for regression testing

This directory contains the validated baseline results for Phase 3 (Braking + ABS + Dugoff tire models) before combined-slip integration begins.

## Contents

- `metadata.json` – Simulation configuration and version information
- `validation_summary.json` – Results of all Phase 3.0 / 3.2 / 3.3 validations
- `comparison.csv` – Locked-wheel vs ABS comparison across tire models
- Future: `.npz` simulation outputs for exact numeric regression

## How to use

Any future change to the tire model, ABS controller, or braking physics should be compared against this baseline to distinguish expected improvements from unintended regressions.

## Freeze Tag

Corresponding Git tag: `v0.3.3-phase3-frozen`
