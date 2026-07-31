# Vehicle Dynamics Simulation Framework

Modular, validated vehicle dynamics library: suspension, braking, ABS, combined-slip tires, bicycle dynamics, and combined braking + steering.

**Repository:** https://github.com/kiingMohlala/vehicledynamics

## Current Status

| Phase | Description | Status |
|-------|-------------|--------|
| 3.x | Braking + ABS + Combined-Slip Dugoff | ✅ Validated |
| 4.0 | Dynamic Bicycle Model | ✅ Implementation Validated |
| 4.1 | Load Transfer Diagnostics | ✅ Implementation Validated |
| **4.2** | **Combined Braking & Steering** | ✅ **Implementation Validated** |

## Progression

Quarter-car suspension → Braking → ABS → Combined-slip tires → Bicycle dynamics → Load-transfer diagnostics → Combined braking & steering

## Next

**Phase 5.0 – Dual-Track Vehicle Model (4-wheel)**  
Independent FL/FR/RL/RR wheels, true yaw moments from longitudinal forces, foundation for ESC.

## Quick Start

```bash
pip install -r requirements.txt
python -m vehicle_dynamics.braking.validation
python -m vehicle_dynamics.tire.validation_combined
python -m vehicle_dynamics.lateral.validation
python -m vehicle_dynamics.combined.validation
```

## License

MIT
