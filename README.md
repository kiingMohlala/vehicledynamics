# Vehicle Dynamics Simulation Framework

Modular, validated vehicle dynamics library covering suspension, braking, ABS, and tire models.

**Repository:** https://github.com/kiingMohlala/vehicledynamics

## Project Structure

```
vehicle_dynamics/
├── braking/                 # Phase 3 – Longitudinal braking + ABS + thermal
│   ├── parameters.py
│   ├── weight_transfer.py
│   ├── brake_torque.py
│   ├── wheel_dynamics.py
│   ├── thermal.py
│   ├── abs_controller.py
│   ├── simulation.py
│   ├── result.py
│   └── validation.py
├── tire/                    # Phase 3.3 / 3.4 – Dugoff tire models
│   ├── base.py
│   ├── dugoff.py             # Combined-slip capable
│   ├── factory.py
│   └── validation_combined.py
├── docs/
└── requirements.txt
```

## Current Status

| Phase | Description                              | Status                          |
|-------|------------------------------------------|---------------------------------|
| 3.0   | Braking Dynamics + Thermal               | Validated                       |
| 3.2   | ABS Controller                           | Validated                       |
| 3.3   | Standard Dugoff (longitudinal)           | Implementation Validated        |
| 3.4   | Combined-Slip Dugoff                     | Validation in progress          |

## Quick Start

```bash
pip install -r requirements.txt

# Run braking validation
python -m vehicle_dynamics.braking.validation

# Run combined-slip tire validation
python -m vehicle_dynamics.tire.validation_combined
```

## Key Design Principles

- Selectable tire models via dependency injection
- Independent validation before system integration
- Clear separation of implementation validation vs physical validation
- Frozen public interfaces (`TireModel`, `BrakeSimulation`, `BrakeSimulationResult`)

## Next Steps

- Complete Phase 3.4.2 combined-slip validation
- Visualization of Fx / Fy / utilization surfaces
- Full regression against Phase 3 baseline
- Combined-slip integration into BrakeSimulation
- Bicycle model + ESC foundation
