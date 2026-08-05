# Phase 3 Baseline Archive

**Status:** Frozen reference for regression testing

This directory contains the validated baseline results for Phase 3 before and after combined-slip integration.

## Contents

- `metadata.json` – Simulation configuration and version information
- `validation_summary.json` – Results of all Phase 3.0 / 3.2 / 3.3 / 3.4 validations
- `comparison.csv` – Locked-wheel vs ABS comparison across tire models
- `plots/` – Combined-slip surface visualizations + visual checklist

## Freeze Tags

- `v0.3.3-phase3-frozen` – Pre-combined-slip baseline
- `v0.3.4-phase3.4-combined-slip` – Combined-slip integrated & regression validated

## How to use

Any future change to the tire model, ABS controller, or braking physics should be compared against this baseline to distinguish expected improvements from unintended regressions.
