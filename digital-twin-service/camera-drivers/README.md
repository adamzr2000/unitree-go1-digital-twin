# astra ROS Drivers. 

## What is it?



## What does this container do?

This container runs the ROS drivers for the Depth Camera Orbbec Astra S

## Run it?

### Dependencies:

The astra ROS Drivers depends on:
  - ros-master (tutorial [here](../ros-master/))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `astra-camera` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example
In this folder we also provide a docker run example. 

> Note: ensure that the astra camera is connected via USB to your PC before testing this container.

To run the astra ROS Drivers:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `astra-camera` container up and running.

