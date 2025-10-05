#!/bin/bash

# Assemble docker image. 
echo 'Building monitoring-agent docker image.'
sudo docker build . -t monitoring-agent
