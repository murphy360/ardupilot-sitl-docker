#!/bin/bash

base_image_name="ardupilot"

# Argument check (Accepts branch name as an argument, defaults to main)
branch=${1:-main}

function print_section() {
    printf "\n\n\n***************************************************\n"
    printf "$1\n"
    printf "***************************************************\n\n\n"
}

# Checkout to the specified branch
print_section "Checking out to the specified branch..."
git fetch
git checkout $branch

# git pull
print_section "Pulling the latest changes from the repository..."
git pull

# Source the build script
source ./build_images.sh