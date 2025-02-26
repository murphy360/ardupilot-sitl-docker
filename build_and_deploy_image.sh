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
    print "Processing directory: $dir"
    # get directory name without trailing slash
    dir_string=${dir%/}
    printf "Direcotry name ${dir_string}\n\n"

    if [ -f "$dir/Dockerfile" ]; then
        image_name="${base_image_name}_$($dir)"
        
        # Stop and remove the Docker container
        print_section "Stopping and removing the Docker container for $image_name..."
        docker compose -f $dir/docker-compose.yml down
        docker container ls -a | grep $image_name | awk '{print $1}' | xargs docker container rm

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