# D435i ROS Drivers. 

## What is it?



## What does this container do?

This container runs the ROS drivers for the Depth Camera D435 - Intel RealSense.

## Run it?

### Dependencies:

The D435i ROS Drivers depends on:
  - ros-master (tutorial [here](../ros-master/))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `d435i-camera` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example
In this folder we also provide a docker run example. 

> Note: ensure that the d435i camera is connected via USB to your PC before testing this container.

To run the d435i ROS Drivers:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `d435i-camera` container up and running.

