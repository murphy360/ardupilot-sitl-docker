ArduPilot SITL Docker Images
============================

[![CI](https://github.com/murphy360/ardupilot-sitl-docker/actions/workflows/ci.yml/badge.svg)](https://github.com/murphy360/ardupilot-sitl-docker/actions/workflows/ci.yml)

Ready-to-run [ArduPilot](https://github.com/ArduPilot/ardupilot) Software-in-the-Loop simulators, one image per vehicle, built from ArduPilot's official stable releases.

ArduPilot publishes CI build environments and a development container, but no runnable SITL image. This repo is the thin layer on top: it runs ArduPilot's own `install-prereqs-ubuntu.sh`, builds a single vehicle from a release tag, and starts `sim_vehicle.py` with the MAVLink TCP server on port 5760.

Images
------

Multi-arch (`linux/amd64`, `linux/arm64`) images are published to GitHub Container Registry:

| Vehicle | Image | ArduPilot | Default `VEHICLE` / `MODEL` |
|---------|-------|-----------|-----------------------------|
| Copter  | `ghcr.io/murphy360/ardupilot-sitl-docker-copter` | Copter-4.7.1 | `ArduCopter` / `+` |
| Plane   | `ghcr.io/murphy360/ardupilot-sitl-docker-plane`  | Plane-4.7.1  | `ArduPlane` / `plane` |
| Rover   | `ghcr.io/murphy360/ardupilot-sitl-docker-rover`  | Rover-4.7.1  | `Rover` / `rover` |
| Sub     | `ghcr.io/murphy360/ardupilot-sitl-docker-sub`    | Sub-4.7.1    | `ArduSub` / `vectored` |

Tags: `latest` (master), the short commit SHA, and `X.Y.Z` for `vX.Y.Z` git tags of this repo.

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
| `ALT`      | `14`       | Home altitude (m) |
| `DIR`      | `270`      | Home heading (deg) |
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

The single [Dockerfile](Dockerfile) builds any vehicle. It defaults to Copter:

```bash
docker build --tag ardupilot-copter .
```

Pick another vehicle or release with build args (CI's per-vehicle values are in [.github/workflows/ci.yml](.github/workflows/ci.yml)):

```bash
docker build --tag ardupilot-plane \
   --build-arg GIT_TAG=Plane-4.7.1 \
   --build-arg WAF_TARGET=plane \
   --build-arg SIM_VEHICLE=ArduPlane \
   --build-arg SIM_FRAME=plane \
   .
```

Updating ArduPilot
------------------

Bump the `GIT_TAG` values in [.github/workflows/ci.yml](.github/workflows/ci.yml) (and the Copter default in the [Dockerfile](Dockerfile)) to the new release tags. CI builds every vehicle on both architectures and smoke-tests that the simulator starts before anything is published. Dependabot keeps the Ubuntu base image and the GitHub Actions current.

Originally forked from [radarku/ardupilot-sitl-docker](https://github.com/radarku/ardupilot-sitl-docker).
