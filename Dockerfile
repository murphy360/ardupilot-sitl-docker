FROM ubuntu:18.04

ARG COPTER_TAG=Copter-4.0.3

# Trick to get apt-get to not prompt for timezone in tzdata
ARG DEBIAN_FRONTEND="noninteractive"

# Update and upgrade the base system
RUN apt-get update -y

# Install git and set it to use https instead of git protocol
RUN apt-get install -y git gitk; git config --global url."https://github.com/".insteadOf git://github.com/

# Now grab ArduPilot from GitHub
RUN git clone --recurse-submodules https://github.com/your-github-userid/ardupilot /ardupilot

# Set the working directory
WORKDIR /ardupilot

# Checkout the latest Copter...
RUN git checkout ${COPTER_TAG}

# Need sudo and lsb-release for the installation prerequisites
# gdb is for debugging
RUN apt-get install -y sudo lsb-release tzdata gdb

# Need USER set so usermod does not fail...
# Install all prerequisites now. This script is from ardupilot repository
RUN USER=nobody Tools/environment_install/install-prereqs-ubuntu.sh -y 

# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
#RUN ./waf clean
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