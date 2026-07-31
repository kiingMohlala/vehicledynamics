# Vehicle Dynamics Simulation Framework

Modular, validated vehicle dynamics library covering suspension, braking, ABS, and tire models.

**Repository:** https://github.com/kiingMohlala/vehicledynamics

## Current Status

| Phase | Description | Status |
|-------|-------------|--------|
| 3.0 | Braking Dynamics + Thermal | ✅ Validated |
| 3.2 | ABS Controller | ✅ Validated |
| 3.3 | Standard Dugoff (longitudinal) | ✅ Implementation Validated |
| **3.4** | **Combined-Slip Dugoff** | ✅ **Integrated & Regression Validated** |

## Project Structure

```
vehicle_dynamics/
├── braking/                 # Longitudinal braking + ABS + thermal
├── tire/                    # Dugoff tire models (longitudinal + combined-slip)
├── docs/                    # Phase status, milestones, integration plans
└── baseline/phase3/         # Frozen regression baselines + plots
```

## Quick Start

```bash
pip install -r requirements.txt

# Run braking validation
python -m vehicle_dynamics.braking.validation

# Run combined-slip tire validation
python -m vehicle_dynamics.tire.validation_combined

# Generate combined-slip surfaces
python -m vehicle_dynamics.tire.visualization
```

## Key Design Principles

- Selectable tire models via dependency injection
- Independent validation before system integration
- Clear separation of implementation validation vs physical validation
- Frozen public interfaces
- Regression baselines for every major milestone

## Next Steps

- Bicycle model (lateral dynamics)
- Steering-while-braking scenarios
- Electronic Stability Control (ESC) foundation

## License

MIT
