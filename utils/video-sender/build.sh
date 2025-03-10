#!/bin/bash

#Assemble docker image. 
echo 'Building video-sender docker image.'
sudo docker build . -t video-sender
