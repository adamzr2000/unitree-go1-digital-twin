#!/bin/bash

container_image="vxlan-configurator"

echo "Building $container_image docker image."
sudo docker build . -t $container_image
