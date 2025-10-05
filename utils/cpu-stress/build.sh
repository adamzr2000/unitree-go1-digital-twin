#!/bin/bash

container_image="cpu-stress"

echo "Building $container_image docker image."
sudo docker build . -t $container_image
