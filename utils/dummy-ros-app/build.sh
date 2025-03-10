#!/bin/bash

# Assemble docker image. 
echo 'Building ros-app docker image.'

docker build -t ros-app .

