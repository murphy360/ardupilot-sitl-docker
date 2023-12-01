FROM ardupilot/ardupilot-dev-base

USER ardupilot
ARG COPTER_TAG=Copter-4.4.3
ARG DEBIAN_FRONTEND="noninteractive"

# Now grab ArduPilot from GitHub
# j8 is for parallelism
RUN git clone --recurse-submodules -j8 https://github.com/ArduPilot/ardupilot.git /ardupilot

# Set the working directory
WORKDIR /ardupilot

# Checkout the latest Copter...
RUN git checkout ${COPTER_TAG}

# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter
RUN ./waf rover 
RUN ./waf plane
RUN ./waf sub

# TCP 5760 is what the sim exposes by default
EXPOSE 5760/tcp

# Variables for simulator
ENV INSTANCE 0
ENV LAT 42.3898
ENV LON -71.1476
ENV ALT 14
ENV DIR 270
ENV MODEL +
ENV SPEEDUP 1
ENV VEHICLE ArduCopter

# Finally the command
ENTRYPOINT /ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --speedup ${SPEEDUP}