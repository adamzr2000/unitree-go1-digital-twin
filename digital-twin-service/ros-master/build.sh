#!/bin/bash

container_image="ros-master"
 
echo "Building $container_image docker image."

sudo docker build . -t $container_image