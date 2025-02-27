#!/bin/bash

rm -r ardupilot

# Clone the ArduPilot repository with submodules
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Build the Docker image
docker build . -t ardupilot

# Run the Docker container
docker run --rm -it -v "$(pwd):/ardupilot" -u "$(id -u):$(id -g)" ardupilot:latest bash
