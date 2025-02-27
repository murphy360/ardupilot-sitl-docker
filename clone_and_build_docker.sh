#!/bin/bash

# Clone the ArduPilot repository with submodules
git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
cd ardupilot

# Build the Docker image
docker build . -t ardupilot --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)

# Run the Docker container
docker run --rm -it -v "$(pwd):/ardupilot" -u "$(id -u):$(id -g)" ardupilot:latest bash
