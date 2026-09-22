FROM ubuntu:24.04

# Which ArduPilot release to build, and which waf target to build from it.
# CI resolves each vehicle's moving <Vehicle>-stable tag to its release tag
# (e.g. Copter-4.7.1) and passes that; the defaults build the latest stable
# Copter. Locally, a moved stable tag needs --no-cache to be picked up.
ARG GIT_TAG=ArduCopter-stable
ARG WAF_TARGET=copter
# Runtime defaults baked into the image (overridable with `docker run --env`).
ARG SIM_VEHICLE=ArduCopter
ARG SIM_FRAME=+
# Start altitude (m AMSL). Sub must be 0: the simulated depth sensor measures
# from 0 m AMSL, so anything else starts it above/below the water surface.
ARG SIM_ALT=14

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000

# ubuntu:24.04 ships a default "ubuntu" user on UID/GID 1000; drop it so the
# ardupilot user can take that ID.
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd ${USER_NAME} --gid ${USER_GID} \
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

RUN apt-get update && apt-get install --no-install-recommends -y \
    ca-certificates \
    git \
    lsb-release \
    sudo \
    tini \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# install-prereqs-ubuntu.sh apt-installs via sudo, so the build user needs it
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && mkdir -p /${USER_NAME} \
    && chown ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}
WORKDIR /${USER_NAME}

RUN git clone --depth 1 --branch ${GIT_TAG} https://github.com/ArduPilot/ardupilot.git . \
    && git submodule update --init --recursive --depth 1

# ArduPilot's own dependency installer, trimmed to what SITL needs: no GUI
# (MAVProxy map/console), no coverage tools, no STM32 firmware toolchain.
RUN SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 \
    DO_AP_STM_ENV=0 DO_PYTHON_VENV_ENV=1 AP_DOCKER_BUILD=1 USER=${USER_NAME} \
    Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# The prereqs script puts pymavlink, pexpect, etc. in this venv; putting it on
# PATH is what `source venv/bin/activate` would do, minus needing a login shell.
ENV VIRTUAL_ENV=/home/${USER_NAME}/venv-ardupilot
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

# Build only this image's vehicle, then keep just its binary: sim_vehicle.py
# runs with --no-rebuild so the object files and ccache are dead weight.
RUN ./waf configure --board sitl \
    && ./waf ${WAF_TARGET} \
    && find build/sitl -mindepth 1 -maxdepth 1 ! -name bin -exec rm -rf {} + \
    && rm -rf ~/.ccache

# sim_vehicle.py backgrounds the simulator with its output sent to
# /tmp/<Vehicle>.log (no terminal to open a window in). Point those files at
# PID 1's stdout so the simulator shows up in `docker logs`.
RUN for v in ArduCopter ArduPlane Rover ArduSub; do ln -sf /proc/1/fd/1 /tmp/$v.log; done

# Instance will offset your port number by 10 https://discuss.ardupilot.org/t/multiple-sitl-instance-help/65283
ENV INSTANCE=0 \
    LAT=42.3898 \
    LON=-71.1476 \
    ALT=${SIM_ALT} \
    DIR=270 \
    MODEL=${SIM_FRAME} \
    SPEEDUP=1 \
    VEHICLE=${SIM_VEHICLE}

EXPOSE 5760/tcp

COPY --chmod=755 entrypoint.sh /usr/local/bin/sitl-entrypoint.sh

# tini as PID 1 forwards `docker stop` to the whole process group (-g) and
# reaps zombies; sim_vehicle.py as PID 1 ignores SIGTERM and gets SIGKILLed.
ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/usr/local/bin/sitl-entrypoint.sh"]
