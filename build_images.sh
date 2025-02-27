#!/bin/bash

# Function to print section headers
function print_section() {
    printf "\n\n\n***************************************************\n"
    printf "$1\n"
    printf "***************************************************\n\n\n"
}

print_section "Stopping Running Containers with Docker Compose"
docker compose down

# Iterate through directories with Dockerfiles and build each image
for dir in */ ; do

    print_section "Processing directory: ${dir}"

    lower_case_directory_name="${dir%"${dir##*[!/]}"}" # extglob-free multi-trailing-/ trim
    lower_case_directory_name="${lower_case_directory_name##*/}"  # remove everything before the last /
    lower_case_directory_name="${lower_case_directory_name:-/}"   # correct for dirname=/ case
    lower_case_directory_name="${lower_case_directory_name,,}"  # convert to lowercase

    if [ -f "$dir/Dockerfile" ]; then
        
        image_name="${base_image_name}_${lower_case_directory_name}"

        container_id=$(docker container ls -a | grep $image_name | awk '{print $1}') 


        docker stop $image_name
        
        if [ -n "$container_id" ]; then 
            print_section "Stopping and removing the Docker container for ${container_id}..."
            docker container rm $container_id
        else
            printf "nable to find container ${container_id} for ${image_name}\n\n"
        fi

        image=$(docker image ls | grep $image_name | awk '{print $3}')
        if [ -n "$image" ]; then
            print_section "Removing the Docker image for ${image_name}..."
            docker image rm $image
        else
            printf "Error: Unable to find image for $image_name\n\n"
        fi
        
        # Build the Docker image
        print_section "Building the Docker image for $image_name..."
        docker build --progress=plain -t $image_name $dir

        docker image ls | grep $image_name
    else
        printf "No Dockerfile found in ${dir}... Continuing\n\n"
    fi
done

# Run Docker Compose in detached mode
print_section "Running Docker Compose in detached mode"
docker compose up -d
