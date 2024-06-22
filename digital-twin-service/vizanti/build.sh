#!/bin/bash

#Assemble docker image. 
echo 'Building go1-vizanti docker image.'
sudo docker build . -t go1-vizanti
