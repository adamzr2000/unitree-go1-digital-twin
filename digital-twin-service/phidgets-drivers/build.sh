#!/bin/bash

container_image="phidgets-imu:latest"

# Assemble docker image. 
echo "Building $container_image docker image."
sudo docker build . -t $container_image
