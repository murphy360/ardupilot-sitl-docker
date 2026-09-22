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
