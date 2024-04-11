#!/bin/bash

#Assemble docker image. 
echo 'Building roscore docker image.'
sudo docker build . -t go1-roscore
