# Project Milestones

## v0.3.4-phase3.4-combined-slip (2026-07-31)

**Status:** Frozen – Integrated & Regression Validated

### Included
- Phase 3.0 Braking Dynamics (validated)
- Phase 3.2 ABS Controller (validated)
- Phase 3.3 Standard Dugoff longitudinal tire model (implementation validated)
- Phase 3.4 Combined-Slip Dugoff (numerical + visual + regression validated)
- Selectable tire model architecture
- Independent validation suites
- Cross-model comparison baseline
- Combined-slip surfaces and visual checklist

### Explicit Limitations
- Pure longitudinal vehicle dynamics only (no bicycle model yet)
- Small-angle lateral stiffness approximation
- No experimental tire data comparison
- No relaxation length / transient tire dynamics

### Recommended Git Tag
```bash
git tag -a v0.3.4-phase3.4-combined-slip -m "Phase 3.4 Combined-Slip Dugoff: Integrated & Regression Validated"
git push origin v0.3.4-phase3.4-combined-slip
```

## Previous Freeze

### v0.3.3-phase3-frozen (2026-07-31)
Braking + ABS + Standard Dugoff (longitudinal only)
