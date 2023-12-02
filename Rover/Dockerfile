FROM ubuntu:22.04


ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ENV CCACHE_MAXSIZE=1G
ARG GIT_TAG=Master
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 

# Add non root username with sudo access
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

# Install dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    bash-completion \ 
    git \
    apt-utils \
    ca-certificates \
    rsync

# Create non root user
ENV USER=${USER_NAME}

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}
RUN mkdir -p /${USER_NAME}
RUN chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

# Switch to non root user
USER ${USER_NAME}

# add waf alias to ardupilot waf to .ardupilot_env
RUN echo "alias waf=\"/${USER_NAME}/waf\"" >> ~/.ardupilot_env

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Create entrypoint as docker cannot do shell substitution correctly
RUN export ARDUPILOT_ENTRYPOINT="/home/${USER_NAME}/ardupilot_entrypoint.sh" \
    && echo "#!/bin/bash" > $ARDUPILOT_ENTRYPOINT \
    && echo "set -e" >> $ARDUPILOT_ENTRYPOINT \
    && echo "source /home/${USER_NAME}/.ardupilot_env" >> $ARDUPILOT_ENTRYPOINT \
    && echo 'exec "$@"' >> $ARDUPILOT_ENTRYPOINT \
    && chmod +x $ARDUPILOT_ENTRYPOINT \
    && sudo mv $ARDUPILOT_ENTRYPOINT /ardupilot_entrypoint.sh

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

# Now grab ArduPilot from GitHub
RUN git clone https://github.com/ArduPilot/ardupilot.git /${USER_NAME}

# Set the working directory
WORKDIR /${USER_NAME}

# Checkout the latest Copter...
RUN git checkout ${GIT_TAG}

# Now start build instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Install the required packages
RUN /${USER_NAME}/Tools/environment_install/install-prereqs-ubuntu.sh -y

# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter
RUN ./waf rover 
RUN ./waf plane
RUN ./waf sub

# Instance will offset your port number by 10 https://discuss.ardupilot.org/t/multiple-sitl-instance-help/65283
ENV INSTANCE 0
ENV LAT 42.3898
ENV LON -71.1476
ENV ALT 14
ENV DIR 270
ENV MODEL +
ENV SPEEDUP 1
ENV VEHICLE ArduCopter

# Cleanup
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Finally the command
ENTRYPOINT /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --speedup ${SPEEDUP}