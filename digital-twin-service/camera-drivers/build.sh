#!/bin/bash

container_image="astra-camera"

echo "Building $container_image docker image."
sudo docker build . -t $container_image