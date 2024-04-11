#!/bin/bash

# Assemble docker image. 
echo 'Building rplidar-lidar docker image.'
sudo docker build . -t rplidar-lidar
