# The torpedo front seat: a simulator stand-in that speaks ArduSub's MAVLink.
# Pure Python; it follows no ArduPilot release. A hull comes as a model file
# (schema 1) mounted at run time and named by TORPEDO_MODEL.
FROM python:3.12-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends tini \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml /src/pyproject.toml
COPY torpedo_sitl /src/torpedo_sitl
RUN pip install --no-cache-dir /src && rm -rf /src

COPY entrypoint.sh /entrypoint.sh

# The SITL images' environment names (INSTANCE offsets the ports by 10 each)
ENV INSTANCE=0 \
    LAT=42.3898 \
    LON=-71.1476 \
    ALT=0 \
    DIR=270 \
    SPEEDUP=1 \
    SYSID=1 \
    SITL_PARAMS="" \
    BATT_CAPACITY=10000 \
    TORPEDO_MODEL=""

# SERIAL0 (Proteus), SERIAL1 (QGroundControl), SERIAL2 (the referee, the world)
EXPOSE 5760 5762 5763

ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/entrypoint.sh"]
