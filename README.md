ArduPilot SITL Docker Images
============================

[![CI](https://github.com/murphy360/ardupilot-sitl-docker/actions/workflows/ci.yml/badge.svg)](https://github.com/murphy360/ardupilot-sitl-docker/actions/workflows/ci.yml)

Ready-to-run [ArduPilot](https://github.com/ArduPilot/ardupilot) Software-in-the-Loop simulators, one image per vehicle, built from ArduPilot's latest official stable release and rebuilt automatically when a new one ships.

ArduPilot publishes CI build environments and a development container, but no runnable SITL image. This repo is the thin layer on top: it runs ArduPilot's own `install-prereqs-ubuntu.sh`, builds a single vehicle from a release tag, and starts `sim_vehicle.py` with the MAVLink TCP server on port 5760.

Images
------

Multi-arch (`linux/amd64`, `linux/arm64`) images are published to GitHub Container Registry:

| Vehicle | Image | Tracks | Default `VEHICLE` / `MODEL` / `ALT` |
|---------|-------|--------|-------------------------------------|
| Copter  | `ghcr.io/murphy360/ardupilot-sitl-docker-copter` | `ArduCopter-stable` | `ArduCopter` / `+` / `14` |
| Plane   | `ghcr.io/murphy360/ardupilot-sitl-docker-plane`  | `ArduPlane-stable`  | `ArduPlane` / `plane` / `14` |
| Rover   | `ghcr.io/murphy360/ardupilot-sitl-docker-rover`  | `APMrover2-stable`  | `Rover` / `rover` / `14` |
| Sub     | `ghcr.io/murphy360/ardupilot-sitl-docker-sub`    | `ArduSub-stable`    | `ArduSub` / `vectored` / `0` |

Tags:
- `latest`: the current ArduPilot stable release.
- `X.Y.Z` / `X.Y`: a specific ArduPilot release (e.g. `4.7.1`, `4.7`). Pin one of these if you don't want to move when ArduPilot does.
- short commit SHA of this repo.

Each image contains only its own vehicle, so `VEHICLE` must stay on that image's vehicle. `MODEL` can be any frame for that vehicle (see below).

Quick Start
-----------

```bash
docker run -it --rm -p 5760:5760 ghcr.io/murphy360/ardupilot-sitl-docker-copter
```

Then connect a ground station or MAVProxy to TCP port 5760:

```bash
mavproxy.py --master=tcp:localhost:5760
```

The simulator's console output goes to `docker logs`.

Options
-------

| Variable   | Default (Copter) | Meaning |
|------------|------------------|---------|
| `INSTANCE` | `0`        | SITL instance number; offsets the ports by 10 × instance |
| `LAT`      | `42.3898`  | Home latitude |
| `LON`      | `-71.1476` | Home longitude |
| `ALT`      | `14`       | Home altitude (m AMSL). Sub defaults to `0`, the water surface |
| `DIR`      | `270`      | Home heading (deg); also the takeoff direction for Plane |
| `MODEL`    | `+`        | Frame, passed to `--frame` |
| `SPEEDUP`  | `1`        | Simulation speed multiplier |
| `VEHICLE`  | `ArduCopter` | Passed to `--vehicle`; leave at the image default |

For example, a skid-steer rover somewhere else, running at 2× speed:

```bash
docker run -it --rm -p 5761:5760 \
   --env MODEL=rover-skid \
   --env LAT=39.9656 \
   --env LON=-75.1810 \
   --env ALT=276 \
   --env DIR=180 \
   --env SPEEDUP=2 \
   ghcr.io/murphy360/ardupilot-sitl-docker-rover
```

The same settings can live in an env file (see [env.list](env.list)):

```bash
docker run -it --rm -p 5761:5760 --env-file env.list ghcr.io/murphy360/ardupilot-sitl-docker-rover
```

Some frames per vehicle (run `sim_vehicle.py --help` inside an image for the full list):

```
ArduCopter: +, X, quad, hexa, octa, octa-quad, y6, tri, heli, heli-dual, coaxcopter, singlecopter
Rover:      rover, rover-skid, balancebot, sailboat, motorboat
ArduPlane:  plane, plane-elevon, plane-vtail, plane-tailsitter, quadplane, quadplane-tilttri
ArduSub:    vectored, vectored_6dof
```

Moving the start position at runtime (MAVLink)
----------------------------------------------

The start position lives in ArduPilot's `SIM_OPOS_*` parameters, so a MAVLink client can move a running simulator without restarting its container. Proteus uses this to put a vehicle at a mission's launch site:

1. `PARAM_SET` the new start position:

   | Parameter | Meaning |
   |-----------|---------|
   | `SIM_OPOS_LAT` | Latitude (deg) |
   | `SIM_OPOS_LNG` | Longitude (deg) |
   | `SIM_OPOS_ALT` | Altitude (m AMSL). **Sub: `0`**, the water surface; the simulated depth sensor measures from 0 m AMSL |
   | `SIM_OPOS_HDG` | Heading (deg); Plane takes off in this direction |

2. While disarmed, send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with `param1=1`.

The simulator restarts in place, and the vehicle and its home come back at the new position and heading. The MAVLink TCP port (5760 + 10 × `INSTANCE`) drops for about 2–3 seconds and then accepts connections again, with no container restart needed. A client can detect a SITL by reading `SIM_OPOS_LAT`; the parameter only exists on simulators.

Measured from sending the reboot until the vehicle arms (ArduPilot 4.7.1, `SPEEDUP=1`, arm retried every second):

| Vehicle | Heartbeat back | Position fix | Can arm |
|---------|---------------:|-------------:|--------:|
| Copter (GUIDED) | ~3 s | ~21 s | ~43 s |
| Plane  | ~2 s | ~11 s | ~24 s |
| Rover  | ~3 s | ~11 s | ~24 s |
| Sub    | ~2 s | ~20 s | ~20 s |

Leave headroom for slower hosts: a 90 s timeout for "can arm" is comfortable. The heading reported in `ATTITUDE` is the flight controller's estimate and takes about a minute after boot to settle within a few degrees of the true heading (which `SIMSTATE` reports).

**Restarting the container resets the position.** The entrypoint starts the simulator with `-w` (wipe saved parameters) and loads `SIM_OPOS_*` from `LAT`/`LON`/`ALT`/`DIR`, so a `docker restart` always comes back at the env location. A MAVLink reboot keeps saved parameters, which is why the new position sticks until then.

[tests/sitl_relocate.py](tests/sitl_relocate.py) runs this whole sequence inside a container, and CI runs it against every image. From a checkout of this repo:

```bash
docker run -d --name sitl -v "$PWD/tests:/tests:ro" ghcr.io/murphy360/ardupilot-sitl-docker-copter
docker exec sitl python3 /tests/sitl_relocate.py relocate 36.6021 -121.8947 5 90 --arm
```

The torpedo (not ArduPilot)
---------------------------

A torpedo cannot hover or move sideways. It holds speed, heading and depth on its fins and a stern thruster, follows lines and flies circles. ArduSub's position controller assumes a vehicle that can hover and push sideways, so a torpedo flown by it steers badly: it swings in S-tracks, overshoots the end of a lane and settles slowly.

`torpedo/` is a torpedo's own front seat for a simulator: its physics, control and guidance (pure Python 3.12, no dependencies), and a MAVLink layer in front of them that speaks ArduSub's dialect, so a ground station or a vehicle interface that flies an ArduSub submarine flies it unchanged. Physics, control and MAVLink run in one process, so it runs at 30 times real time and more.

| Module | What it is |
|--------|------------|
| `torpedo_sitl/model.py` | `load(path)`: a hull's model file (schema 1, JSON); `ModelError` names a missing key |
| `torpedo_sitl/physics.py` | `Hull`: five degrees of freedom, added mass, quadratic drag, fin lift, buoyancy, thrust that never reverses |
| `torpedo_sitl/control.py` | Three loops: speed, heading, depth |
| `torpedo_sitl/guidance.py` | Lines (line of sight), corners, points and circles |
| `torpedo_sitl/autopilot.py` | The loops and the guidance together |
| `torpedo_sitl/check.py` | `torpedo-check <model.json>`: flies the acceptance scenarios on any model file |
| `torpedo_sitl/vehicle.py` | The vehicle: modes, arming, targets, battery, what it knows of itself |
| `torpedo_sitl/server.py`, `link.py`, `commands.py`, `messages.py` | The three MAVLink ports, the clock, each connection's stream rates, the commands and the telemetry |
| `torpedo_sitl/params.py` | The parameter store and its defaults |
| `torpedo_sitl/mission.py`, `auto.py` | The mission and its protocol; AUTO flying it |
| `torpedo_sitl/failsafe.py` | The failsafes and the Proteus-dead script |
| `torpedo_sitl/world.py` | The simulated world: current, GPS, DVL, rangefinder, depth sensor |
| `torpedo_sitl/captured.py`, `ardusub_surface.json` | What the real ArduSub says, captured |
| `torpedo_sitl/cli.py` | `torpedo-sitl`: starts the front seat from the SITL images' environment |

A hull comes as a model file mounted at run time. The repository holds none of a real vehicle's: its tests fly two generic hulls, `torpedo-test-small` and `torpedo-test-large`, written by `torpedo/tests/models/make_fixtures.py`. A hull is known only by its id (`torpedo-100`, `torpedo-300`, and so on).

### The image

`ghcr.io/murphy360/ardupilot-sitl-docker-torpedo` (`linux/amd64`, `linux/arm64`; tags `latest`, the short commit sha and `0.1.<run>`) is built from [torpedo/Dockerfile](torpedo/Dockerfile) on `python:3.12-slim`. It follows no ArduPilot release. It carries `torpedo-sitl` (the entrypoint) and `torpedo-check`.

```bash
docker run -it --rm -p 5760:5760 -p 5762:5762 -p 5763:5763 \
   -e SPEEDUP=30 -e TORPEDO_MODEL=/models/torpedo-test-small.json \
   -v "$PWD/torpedo/tests/models:/models:ro" \
   ghcr.io/murphy360/ardupilot-sitl-docker-torpedo
```

With no `TORPEDO_MODEL` (or a file that is not a schema-1 model) the container says why and exits 2.

| Variable | Default | Meaning |
|----------|---------|---------|
| `TORPEDO_MODEL` | none | Path of the hull's model file in the container. Required |
| `INSTANCE` | `0` | Offsets the ports by 10 x instance |
| `LAT`, `LON`, `ALT`, `DIR` | `42.3898`, `-71.1476`, `0`, `270` | Start position (deg), altitude (m AMSL, `0` is the water surface) and heading (deg). They set `SIM_OPOS_*` |
| `SPEEDUP` | `1` | Vehicle time runs this many times wall time. Behind it (a loaded host) the front seat steps as fast as it can and falls behind, as the SITL does |
| `SYSID` | `1` | MAVLink system id (component 1) |
| `SITL_PARAMS` | empty | Parameter lines (`NAME VALUE`, one per line) applied at start. `--param-file` adds files |
| `BATT_CAPACITY` | `10000` | Battery capacity (mAh). It lasts the hull's endurance at cruise |

| Port | Serial | For |
|------|--------|-----|
| `5760 + 10 x INSTANCE` | SERIAL0 | The vehicle interface (Proteus) |
| `5762 + 10 x INSTANCE` | SERIAL1 | QGroundControl |
| `5763 + 10 x INSTANCE` | SERIAL2 | The referee and the world simulator (`SIM_STATE`) |

All three are TCP servers, MAVLink 2, system id `SYSID`, component 1. They open at start: nothing waits for SERIAL0. A connection may close and come back at any time, and each has its own stream rates. A client sees a submarine (`MAV_TYPE_SUBMARINE`, ArduPilot's autopilot id) with a heartbeat every second of vehicle time, and `SIM_OPOS_LAT` answers, which is how a client knows it is talking to a simulator.

### What it does

*Modes* carry ArduSub's numbers (`SET_MODE` or `MAV_CMD_DO_SET_MODE`); any other is refused (`COMMAND_ACK` UNSUPPORTED). A torpedo cannot hover, so each mode that holds does it by circling.

| Mode | Number | On a torpedo |
|------|-------:|--------------|
| MANUAL | 19 | `MANUAL_CONTROL` x is the thrust (0 to 1000), r the rudder, z the elevator (500 neutral). No input for 1 s: thrust 0, fins neutral |
| STABILIZE | 0 | As MANUAL, but z is a pitch demand up to the hull's steepest pitch, held by the elevators |
| ALT_HOLD | 2 | Holds the depth it has. Shallower than 0.5 m: motor off, it floats. Deeper: it circles the point where it entered |
| GUIDED | 4 | A position target (point guidance, then the circle) or a velocity target (course and speed at the depth it holds; 3 s without a new one and it circles where it is). With no target it circles where it entered |
| SURFACE | 9 | The steering speed on its heading, pitched up to rise. At the surface it switches itself to ALT_HOLD with the motor off, as ArduSub does |
| AUTO | 3 | The mission (below). Refused with no mission |

GUIDED and AUTO are refused until the position estimate is valid (5 s of vehicle time after boot), in ArduSub's words (`Mode change failed: Guided requires position`). Arming (`MAV_CMD_COMPONENT_ARM_DISARM`) is not refused, as ArduSub's is not. *Targets* (`SET_POSITION_TARGET_GLOBAL_INT`) are taken in GUIDED in the frames GLOBAL_INT (depth is the home altitude less alt), GLOBAL_RELATIVE_ALT_INT (depth is -alt) and GLOBAL_TERRAIN_ALT_INT (height above a flat 50 m seabed). A position with a velocity of zero, a position alone and a velocity alone are all understood. A held point is circled for as long as it stands: sending it again does not restart it. `MAV_CMD_DO_CHANGE_SPEED` sets the speed, between the hull's steering speed and its top speed.

*Commands*: `COMPONENT_ARM_DISARM`, `DO_SET_MODE`, `DO_CHANGE_SPEED`, `REQUEST_MESSAGE`, `SET_MESSAGE_INTERVAL` and `PREFLIGHT_REBOOT_SHUTDOWN`; each is answered with a `COMMAND_ACK`. `DO_SET_MISSION_CURRENT` makes an item current (in AUTO the vehicle goes to it). `DO_ORBIT`, `NAV_TAKEOFF` and the rest are UNSUPPORTED, as ArduSub answers. A reboot (refused while armed) closes every link and starts again at `SIM_OPOS_*` with the saved parameters, which is how a client moves the simulator (see the section above).

*Telemetry*, in vehicle time, by `SET_MESSAGE_INTERVAL`, by `REQUEST_DATA_STREAM` (ArduPilot's groups) and by `REQUEST_MESSAGE`: `HEARTBEAT`, `GLOBAL_POSITION_INT`, `ATTITUDE`, `SYS_STATUS`, `GPS_RAW_INT`, `EKF_STATUS_REPORT`, `BATTERY_STATUS`, `HOME_POSITION`, `SYSTEM_TIME`, `EXTENDED_SYS_STATE`, `MISSION_CURRENT`, `FENCE_STATUS`, `DISTANCE_SENSOR`, `RANGEFINDER` (the last two when `RNGFND1_TYPE` is set, and nothing past `RNGFND1_MAX`), `STATUSTEXT` and `SIM_STATE` (the truth). `SYSTEM_TIME` carries the boot clock in vehicle time and `TIMESYNC` is answered. `TERRAIN_REPORT` and `WIND` are not sent.

*Parameters*: `PARAM_REQUEST_READ`, `PARAM_REQUEST_LIST` and `PARAM_SET` work on the names a submarine's client reads (identity, arrival radius, failsafes, fence, battery, rangefinder, EKF sources), `SIM_OPOS_LAT/LNG/ALT/HDG`, and the front seat's own: `TPDO_MODEL` (read only: the number in the hull's id, 100 for `torpedo-100`), `TPDO_LOS_LA`, `TPDO_TURN_K` and `TPDO_LOIT_MIN`, the guidance tuning. `WP_RADIUS_M` starts at the hull's turning radius. A `PARAM_SET` is saved to `/tmp/torpedo-<INSTANCE>.parm` and kept over a reboot; the file is cleared when the container starts. A name in `SITL_PARAMS` or a parameter file that nobody knows is kept and answered.

### Captured from the real ArduSub

`torpedo/torpedo_sitl/ardusub_surface.json` is what the real ArduSub says, captured once from its SITL image by `torpedo/tools/capture_ardusub.py` (`python3 torpedo/tools/capture_ardusub.py` on a host with Docker; it starts the image twice at 5x and removes it, and nothing is flown). It holds every parameter's name and default (the front seat starts with them, in ArduSub's order), the `STATUSTEXT` lines for arming, a refused mode, the GCS failsafe and its clearing, the battery failsafes and a lost depth sensor or GPS, and the `EKF_STATUS_REPORT` flags and `SYS_STATUS` sensor bits in three states: a GPS fix, the GPS off with a DVL's velocity, and neither. The front seat says those words and reports those flags, so a client that reads a failsafe's name or an EKF's health is told what ArduSub would tell it. `torpedo/tests/test_ardusub_contract.py` holds it to that.

### Missions

The mission protocol (`MAV_MISSION_TYPE_MISSION`): upload (`MISSION_COUNT`, then `MISSION_REQUEST_INT` and `MISSION_ITEM_INT` or `MISSION_ITEM`), read-back, `MISSION_CLEAR_ALL`, `MISSION_SET_CURRENT`. Item 0 is home, rewritten as ArduPilot does. An item that arrives twice is answered `INVALID_SEQUENCE` and the upload stays open, as ArduSub answers it. Fence and rally uploads are answered `MAV_MISSION_UNSUPPORTED`. The mission survives a reboot; a mission written in AUTO restarts the command. `MISSION_CURRENT` is streamed and sent on change, `MISSION_ITEM_REACHED` for each navigation item reached.

AUTO starts at the current item (item 1 when the mission had ended) and flies the items:

| Item | What it does |
|------|--------------|
| `NAV_WAYPOINT` | A point at the item's depth (`param2` is its acceptance radius). A run of waypoints is one route: the line from the last point (or from where AUTO began), the next line starting early at a corner |
| `DO_CHANGE_SPEED` | The speed, when the item before it is reached |
| `NAV_LOITER_UNLIM` | The circle about the point at the least loiter radius, for good (the return route's end) |
| `NAV_LOITER_TURNS` | `param1` turns of the circle, `param3` its radius in metres (negative: counter-clockwise). A radius under the least loiter radius is raised to it, with a `STATUSTEXT`. Joined on a tangent; reached after the turns |
| `DO_LAND_START` | A marker, passed over |
| any other | Skipped, with a `STATUSTEXT` |

At the end of the mission the vehicle circles the last point.

### Failsafes

* **GCS**: no `HEARTBEAT` from `MAV_GCS_SYSID` (up to `MAV_GCS_SYSID_HI` when set) for `FS_GCS_TIMEOUT` s of vehicle time while armed, after one was heard. `FS_GCS_ENABLE`: 0 none, 1 warn, 2 disarm, 3 ALT_HOLD (it circles at depth), 4 SURFACE. The heartbeat says CRITICAL until the GCS is back.
* **Proteus dead**: `deploy/autopilot/ardusub/proteus_dead.lua` of the Proteus repository, in Python: the same states, timings and texts (`PDEAD: ...`). `PDEAD_ACTION` 1 waits `PDEAD_HOLD_S` s in the GCS failsafe's depth hold, then SURFACE (0: no limit); 2 goes to AUTO from the return route's `DO_LAND_START` (or item 1) and falls back to the timed wait when AUTO is refused. `SCR_ENABLE` reads 1.
* **Battery**: `BATT_LOW_*` and `BATT_CRT_*` (volts below the limit for `BATT_LOW_TIMER` s, or the charge left below the mAh limit); `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT`: 0 none, 1 warn, 2 disarm, 3 surface.
* **Pilot input**: `FS_PILOT_INPUT` (1 warn, 2 disarm) after `FS_PILOT_TIMEOUT` s with no `MANUAL_CONTROL` in MANUAL or STABILIZE. ArduSub's SITL showed nothing for either action under the capture's conditions, so a warning shows nothing here either.
* **Fence and leak**: parameters only. A fence breach (`FENCE_ENABLE`, the circle about home, the depth limit) is shown in `FENCE_STATUS`; ArduSub only logs it.

### The simulated world

A simulator sets `SIM_*` parameters (the container's start-up file or `SITL_PARAMS`, or `PARAM_SET` from a world simulator on SERIAL2). A real autopilot has none of them. **The errors live in this stand-in and its mounted configuration only**: the estimate's error is made here, from these parameters, and nothing a real vehicle loads carries it.

| Parameter | Effect |
|-----------|--------|
| `SIM_WIND_SPD`, `SIM_WIND_DIR` (the direction it comes *from*), `SIM_WIND_T` | The water current, the same at every depth. It moves the hull over the ground |
| `SIM_GPS1_ENABLE` (older: `SIM_GPS_DISABLE`) | The GPS fix. With the fix the estimate is the GPS |
| `SIM_GPS1_GLTCH_X`, `SIM_GPS1_GLTCH_Y` | The fix's offset, in degrees of latitude and longitude |
| `SIM_VICON_TMASK` (0 no lock; 2 or 8 lock), `SIM_VICON_VGLI_X/Y` (m/s north, east), `SIM_VICON_GLIT_X/Y` (m), `SIM_VICON_V_SD` (noise, m/s) | A DVL, fitted when `VISO_TYPE` is set and `EK3_SRC1_VELXY` is 6. With the GPS off and a lock, the estimate dead-reckons on the true velocity plus those errors. With neither it holds its last position, as EKF3 does. A fix snaps it back to the truth |
| `SIM_SONAR_POS_Z` | The rangefinder reads `50 - SIM_SONAR_POS_Z - depth`, the SITL's flat 50 m floor |
| `SIM_BARO_DISABLE` | The depth sensor fails: the estimate's depth freezes, `SYS_STATUS` says unhealthy, `Depth sensor error!` and an armed vehicle in a depth mode goes to MANUAL, as ArduSub does |

The vehicle steers by its estimate. `SIM_STATE` (and `GPS_RAW_INT` with its offset) carry the truth.

### Developing

```bash
cd torpedo
python3 -m venv .venv && .venv/bin/pip install -e '.[test]'
.venv/bin/pytest tests                                         # the unit tests
.venv/bin/torpedo-check tests/models/torpedo-test-small.json   # the acceptance lines on a hull
```

The acceptance lines `torpedo-check` holds a hull to (each is a unit test too): a turn of 90 degrees reaches no more than 20 m past the lane's end; on a lane the cross-track error stays within 2 m and changes sign at most once (no S-track); it is within 2 m of the next lane 30 s after the corner; speed within 0.1 m/s and heading within 2 degrees (rms) on a straight; a ring of 30 m is flown with a median radius error of 2 m or less; a hold ring entered at 1.9 m/s stays within the ring plus 10 m. The full-rudder turning radius is measured as well and printed against the file's.

CI (the `torpedo` and `torpedo-image` jobs) runs the code rules and the tests, builds the image for both architectures, and smoke tests it at 30x: arm, GUIDED to a point 300 m away at 4 m depth and see it within the waypoint radius in under 30 s of wall time; then upload a three-point route, AUTO, and see `MISSION_ITEM_REACHED` for each point. A push to the default branch publishes the image.

Docker Compose
--------------

```yml
services:
  copter:
    image: ghcr.io/murphy360/ardupilot-sitl-docker-copter
    tty: true
    environment:
      - LAT=32.62354
      - LON=-116.9456
    ports:
      - "5760:5760"
  rover:
    image: ghcr.io/murphy360/ardupilot-sitl-docker-rover
    tty: true
    environment:
      - LAT=32.71234
      - LON=-117.22345
    ports:
      - "5761:5760"
  plane:
    image: ghcr.io/murphy360/ardupilot-sitl-docker-plane
    tty: true
    environment:
      - LAT=32.693993
      - LON=-117.205200
    ports:
      - "5762:5760"
  sub:
    image: ghcr.io/murphy360/ardupilot-sitl-docker-sub
    tty: true
    environment:
      - LAT=32.719617
      - LON=-117.222498
    ports:
      - "5763:5760"
```

Building locally
----------------

The single [Dockerfile](Dockerfile) builds any vehicle. It defaults to the latest stable Copter:

```bash
docker build --tag ardupilot-copter .
```

Pick another vehicle or release with build args (CI's per-vehicle values are in [.github/workflows/ci.yml](.github/workflows/ci.yml)). `GIT_TAG` takes a release tag like `Plane-4.7.1` or a moving one like `ArduPlane-stable`; with a moving tag, add `--no-cache` so Docker doesn't reuse an older checkout:

```bash
docker build --tag ardupilot-plane \
   --build-arg GIT_TAG=ArduPlane-stable \
   --build-arg WAF_TARGET=plane \
   --build-arg SIM_VEHICLE=ArduPlane \
   --build-arg SIM_FRAME=plane \
   .
```

For Sub, also pass `--build-arg SIM_ALT=0` so the default start is at the water surface:

```bash
docker build --tag ardupilot-sub \
   --build-arg GIT_TAG=ArduSub-stable \
   --build-arg WAF_TARGET=sub \
   --build-arg SIM_VEHICLE=ArduSub \
   --build-arg SIM_FRAME=vectored \
   --build-arg SIM_ALT=0 \
   .
```

Updating ArduPilot
------------------

Nothing to do. CI checks ArduPilot daily: it resolves each vehicle's `-stable` tag to its release (e.g. `ArduCopter-stable` → `Copter-4.7.1`) and, if that release isn't published here yet, builds every vehicle on both architectures, smoke-tests that each simulator starts, and only then moves `latest`. If a new release fails to build or start, `latest` stays on the previous one. Dependabot keeps the Ubuntu base image and the GitHub Actions current. To force a rebuild, run the CI workflow manually from the Actions tab.

Originally forked from [radarku/ardupilot-sitl-docker](https://github.com/radarku/ardupilot-sitl-docker).
