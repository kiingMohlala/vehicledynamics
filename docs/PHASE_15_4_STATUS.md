# Phase 15.4 Status — Closed without freezing gains

**Date:** 2026-08-15  
**Tag:** `v1.5.4-closed-loop-esc`

| Layer | Status |
|-------|--------|
| 14.9 Passive plant | **FROZEN** |
| 15.1–15.4 ESC architecture | **VALIDATED** |
| ESC baseline controller | **VALIDATED** |
| ESC gains (`K_Mz`, …) | **NOT FROZEN** |

```
K_us = 0.0065  ← frozen plant characterization (14.9.8)
K_Mz = 4000    ← controller-design parameter (candidate only)
```

Default ESC **OFF**. ESC-OFF ≡ passive regression (3.13 / 8.34 s).
