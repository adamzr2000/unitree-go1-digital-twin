#!/bin/bash

#Assemble docker image. 
echo 'Building rviz docker image.'
sudo docker build . -t go1-rviz-vnc
