# Vehicle Dynamics Simulation Framework

Modular, validated vehicle dynamics library covering suspension, braking, ABS, and tire models.

## Project Structure

```
vehicle_dynamics/
├── suspension/          # Phase 1 – 2-DOF quarter-car + Skyhook/Groundhook/Hybrid
├── braking/             # Phase 3 – Longitudinal braking, thermal, ABS
├── tire/                # Phase 3.3/3.4 – Dugoff tire models (longitudinal + combined-slip)
├── baseline/            # Frozen validation baselines
└── docs/                # Notes and milestones
```

## Current Status

| Phase | Description                              | Status                          |
|-------|------------------------------------------|---------------------------------|
| 1.0   | 2-DOF Quarter-Car + Controllers          | Validated                       |
| 3.0   | Braking Dynamics + Thermal               | Validated                       |
| 3.2   | ABS Controller                           | Validated                       |
| 3.3   | Standard Dugoff (longitudinal)           | Implementation Validated        |
| 3.4   | Combined-Slip Dugoff                     | In progress                     |

## Key Features

- Clean modular architecture with dependency injection
- Independent validation suites for each subsystem
- Selectable tire models (Hard saturation / Simplified Dugoff / Standard Dugoff)
- ABS with hysteresis and valve dynamics
- Energy & passivity checks
- Regression baselines

## Getting Started

```bash
pip install numpy scipy matplotlib
python -m vehicle_dynamics.braking.validation
python -m vehicle_dynamics.tire.validation
```

## License

MIT (or your preferred license)
