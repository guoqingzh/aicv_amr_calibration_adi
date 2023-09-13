#!/bin/bash

docker_image="aicv-amr-calib"
docker_container="compose-aicv-amr-calib-1"

echo "Importing calibration repositories"
mkdir src
vcs import src < calib.repos

if [ $? -eq 0 ]; then
    echo "Repositories successfully imported"
else
    echo "Failed to import calibration repositories"
    exit 1
fi


echo "Attempting to build calibration docker"
cd ./docker/compose
docker compose --env-file template.env -f docker-compose.yaml up -d $docker_image

if [ $? -eq 0 ]; then
    echo "Repositories successfully imported"
else
    echo "Failed to import calibration repositories"
    exit 1
fi

docker attach $docker_container

