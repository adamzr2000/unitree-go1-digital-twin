# LiDAR ROS Drivers. 

## What is it?

This repository contains files for basic device management of the 2D Laser Scanner RPLiDAR A3 with ROS.

A LiDAR (Light Detection and Ranging) is a remote sensing technology that measures distances to objects or surfaces by illuminating them with laser light and analyzing the reflected light. It enables the creation of detailed maps by sending out laser pulses and analyzing the light that bounces back.

The data captured by the RPLiDAR A3 is published on the `/scan` ROS topic.

For further information, refer to the documentation available at: [rplidar ros package](https://wiki.ros.org/rplidar)


## What does this container do?

This container runs the ROS drivers for the RPLiDAR A3.

## Run it?

### Dependencies:

The LiDAR ROS Drivers depends on:
  - ros-master (tutorial [here](../ros-master/))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `rplidar-lidar` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Then, you need to apply LiDAR rules to assign a fixed name to the device port.

After building the image, it's necessary to execute the following script:
```bash
./apply_lidar_rules.sh
```

This script copies necessary [rplidar.rules](rules/rplidar.rules) to the `/etc/udev/rules.d/` directory, sets appropriate permissions, and records the changes. It ensures that the device port remains consistent for the `rplidar` name.

### Docker run example
In this folder we also provide a docker run example. 

> Note: ensure that the RPLiDAR A3 is connected via USB to your PC before testing this container.

To run the LiDAR ROS Drivers:
```bash
./run_example.sh --ros-master-uri http://192.168.40.3:11311 --ros-ip 192.168.40.70 --rf2o-laser-odometry true --odom-topic /odom

```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `lidar` container up and running.

