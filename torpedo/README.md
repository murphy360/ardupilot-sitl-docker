# The torpedo front seat

A torpedo's physics, control and guidance for a simulator, behind a MAVLink layer that
speaks ArduSub's dialect. The repository's [README](../README.md) (the torpedo section)
describes the image, its ports, modes, missions, failsafes and simulated world; this
file is the short map for working on the package.

```bash
python3 -m venv .venv && .venv/bin/pip install -e '.[test]'
.venv/bin/pytest tests                                         # the unit tests
.venv/bin/ruff check . && .venv/bin/ruff format --check .      # the code rules
.venv/bin/torpedo-check tests/models/torpedo-test-small.json   # a hull's acceptance lines
```

## The terrain frame

A terrain-frame altitude (`MAV_FRAME_GLOBAL_TERRAIN_ALT` / `_INT`) is a height above the
bed the downward rangefinder senses, as ArduSub 4.7 flies it with `WP_RFND_USE` 1:

| Module | What it does |
|--------|--------------|
| `torpedo_sitl/rangefinder.py` | ArduSub's reading of the rangefinder: fitted (`RNGFND1_TYPE`), its status against `RNGFND1_MIN` / `RNGFND1_MAX`, healthy after three good readings, the range low-passed at 0.25 Hz, the bed (depth plus filtered range). Also the bed's slope, for the depth loop |
| `torpedo_sitl/terrain.py` | The depth for a height above the bed (between the surface and `depth_limit_m`), the terrain failsafe (`FS_TERRAIN_TIMEOUT_MS` 1 s), its recovery (`FS_TERRAIN_RECOVER_TIMEOUT_MS` 10 s, 1.5 s healthy to resume), the `FS_TERRAIN_ENAB` action (0 disarm, 1 hold, 2 surface) and the arming check (`Arm:` / `PreArm: terrain disabled` with the parameter checks on) |
| `torpedo_sitl/auto.py` | A terrain leg asks the depth afresh at every step, so the bed is followed along each line; recovery flies the vehicle while it runs |
| `torpedo_sitl/targets.py` | GUIDED's targets: a terrain-frame position alone is flown over the bed; with a velocity it is dropped, as ArduSub drops it |
| `torpedo_sitl/control.py` | The depth loop takes the bed's slope: the path's pitch fed forward, its rate to the elevators |

A torpedo cannot hover: where ArduSub holds a position (recovery, `FS_TERRAIN_ENAB` 1)
the torpedo circles, as its other holds do. The words, severities and outcomes are held
to `torpedo_sitl/ardusub_terrain.json`, captured from the ArduSub SITL image by
`tools/capture_terrain.py` (`python3 torpedo/tools/capture_terrain.py` on a host with
Docker), in `tests/test_ardusub_contract.py`; `tests/test_terrain.py` flies the rises,
the lost rangefinder, the arming check and the GUIDED targets.

The seabed in a test is emulated as the SITL's: the rangefinder reads
`50 - SIM_SONAR_POS_Z - depth`, so a bed at depth `b` under the hull is
`SIM_SONAR_POS_Z = 50 - b`, changed as the hull goes.
