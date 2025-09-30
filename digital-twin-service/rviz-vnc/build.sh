#!/bin/bash

container_image="rviz-vnc"

echo "Building $container_image docker image."
sudo docker build . -t $container_image