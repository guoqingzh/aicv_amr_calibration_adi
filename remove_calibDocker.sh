#!/bin/bash

docker_image="aicv_amr:multisensor-record-calib"
docker_container="compose-aicv-amr-calib-1"


# TBD: Stop the container before deleting

echo "Removing calibration container ..."
docker rm $docker_container
if [ $? -eq 0 ]; then
    echo "Calibration docker container removed successfully"
else
    echo "Failed to delete calibration docker container"
    exit 1
fi

echo "Removing calibration docker image ..."
docker image rm $docker_image
if [ $? -eq 0 ]; then
    echo "Calibration docker image removed successfully"
else
    echo "Failed to delete calibration docker image"
    exit 1
fi

