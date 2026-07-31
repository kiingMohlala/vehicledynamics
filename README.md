# Vehicle Dynamics Simulation Framework

Modular, validated vehicle dynamics library covering suspension, braking, ABS, tire models, and lateral dynamics.

**Repository:** https://github.com/kiingMohlala/vehicledynamics

## Current Status

| Phase | Description | Status |
|-------|-------------|--------|
| 3.0 | Braking Dynamics + Thermal | ✅ Validated |
| 3.2 | ABS Controller | ✅ Validated |
| 3.3 | Standard Dugoff (longitudinal) | ✅ Implementation Validated |
| 3.4 | Combined-Slip Dugoff | ✅ Integrated & Regression Validated |
| **4.0** | **Dynamic Bicycle Model** | ✅ **Implementation Validated** |

## Project Structure

```
vehicle_dynamics/
├── braking/          # Longitudinal braking + ABS + thermal
├── tire/             # Dugoff tire models (longitudinal + combined-slip)
├── lateral/          # Phase 4.0 dynamic bicycle model
├── docs/
└── baseline/
    ├── phase3/
    └── phase4/
```

## Quick Start

```bash
pip install -r requirements.txt

# Braking validation
python -m vehicle_dynamics.braking.validation

# Combined-slip tire validation
python -m vehicle_dynamics.tire.validation_combined

# Bicycle model validation
python -m vehicle_dynamics.lateral.validation
```

## Key Design Principles

- Selectable tire models via dependency injection
- Independent validation before system integration
- Clear separation of implementation validation vs physical validation
- Frozen public interfaces
- Regression baselines for every major milestone

## Next Steps

- Phase 4.1 – Lateral load transfer / dynamic normal loads
- Phase 4.2 – Combined braking + steering
- Phase 4.3 – ESC foundation

## License

MIT
