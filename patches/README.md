Simulator patches
=================

The Dockerfile applies every `*.patch` here to the ArduPilot tree, in name order,
right after the clone (`git apply`). A patch that no longer applies fails the build,
so a new ArduPilot release that changes the same lines is caught in CI.

| Patch | What it fixes |
|-------|---------------|
| `0001-motorboat-hull-drag.patch` | The `motorboat` model (`libraries/SITL/SIM_Sailboat.cpp`) had drag only along the hull, so a sideways velocity left by a turn was never damped and a boat holding station slid off it (Proteus #864). Adds a lateral hull drag (linear plus quadratic) and, for a boat without a sail, windage: `SIM_WIND_*` now pushes a motorboat while it is armed (a 10 m/s wind: 0.14 m/s beam on, 0.8 m/s end on; Proteus #462). Also cleans `motorboat.parm`: drops the malformed `SIM_TIDE_0.5` line and `SIM_WIND_SPD 3` / `SIM_WIND_T 1` (they had no effect on a boat without a sail; with windage they would push every boat). |

Each patch is a plain `diff -ru` against the release tree (`a/` and `b/` prefixes).
Worth sending upstream: ArduPilot's `Tools/autotest` would accept the same change.
