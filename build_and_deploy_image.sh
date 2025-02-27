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
        

# Iterate through directories with Dockerfiles and build each image
for dir in */ ; do

    print_section "Processing directory: ${dir}"

    result="${dir%"${dir##*[!/]}"}" # extglob-free multi-trailing-/ trim
    printf "${result}\n\n"
    result="${result##*/}"                  # remove everything before the last /
    printf "${result}\n\n"
    result="${result:-/}"                     # correct for dirname=/ case
    printf "${result}\n\n"
    result="${result,,}"
    printf "${result}\n\n"
    printf "Directory name ${result}\n\n"


    if [ -f "$dir/Dockerfile" ]; then
        image_name="${base_image_name}_$($result)"
        printf "Image name: ${image_name}\n\n"
        
        # Stop and remove the Docker container
        print_section "Stopping and removing the Docker container for $image_name..."
        docker down $image_name
        docker container ls -a | grep $image_name | awk '{print $1}' | xargs docker container rm $image_name
        docker image ls | grep $image_name | awk '{print $3}' | xargs docker image rm $image_name


        # Build the Docker image
        print_section "Building the Docker image for $image_name..."
        docker build -t $image_name $dir

        docker image ls | grep $image_name

        

        # Run docker logs -f
        print_section "Running docker logs -f for $image_name..."
        docker logs -f $image_name
    fi
done

# Run Docker Compose in detached mode
print_section "Running Docker Compose in detached mode
docker compose up -d