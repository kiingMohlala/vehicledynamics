# Phase 6.0 Status

## Phase 6.0 – Suspension Geometry Solver

**Status:** Implemented (run validation before freeze)

### Architecture

Independent of vehicle dynamics. Hardpoints → geometry only.

```
vehicle_dynamics/suspension/
├── hardpoints.py      # pickup points
├── geometry.py        # line intersection, IC construction
├── wishbone.py        # KPI, caster, camber, toe, RC, scrub, trail
├── solver.py
├── result.py
├── validation.py
└── __init__.py
```

### Key physics (not midpoint)

Instant center = **intersection of extended upper and lower arm lines** in the YZ plane.

Roll center = intersection of IC–contact_patch line with vehicle centerline.

### Outputs

Camber, toe, caster, KPI, scrub radius, trail, roll center height, swing arm length, arm lengths.

### Run

```bash
python -m vehicle_dynamics.suspension.validation
```

### Freeze target

```
Phase 6.0 – Suspension Geometry Solver: Implementation Validated
Tag: v0.6.0-phase6.0-geometry
```
