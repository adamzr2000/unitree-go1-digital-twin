# Unitree Go1 Gesture Control App.

## What is this?

This repository contains an application that enables the control of the Unitree Go1 quadruped robot using hand gestures.

## What does this container do?

This container captures video stream from the user's camera, detects his hand gestures, and generates control commands to move the robot. The application consists of two main components: 

- **Gesture Detection Script:**
  - Utilizes an OpenCV Python script to read camera frames (webcam or Intel RealSense D435i).
  - Detects hand gestures (move forward, move backward, turn right, turn left, stop).
  - Generates a `command.txt` file containing the detected gesture.

- **ROS Integration Script:**
  - Reads the `command.txt` file with the generated gesture command.
  - Translates the commands into velocity commands over ROS.
  - Sends velocity commands to the Unitree Go1 robot using the `/cmd_vel` topic.

Here is a visual representation of the possible hand gestures and their corresponding control commands:

![Gesture Control Commands](../../images/gesture-control-commands.png)

## Run it?

### Dependencies:

The Unitree Go1 Gesture Control App depends on:
  - ros-master (tutorial [here](../ros-master/))
  - go1-base (tutorial [here](../go1-base/))

> Note: be sure that all the dependencies are running before you run the Unitree Go1 Gesture Control App container

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

> Note: It will take some time to build the image because we have to assemble a ROS noetic image, install the Unitree Go1 ROS dependencies and install the OpenCV modules.

### This will build the `go1-gesture-control` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example
In this folder we also provide a docker run example. 

To run the Unitree Go1 Gesture Control App:
```bash
./run_example_web.sh
```

This will start the gesture control app on a Flask web server.

Verify that the container is up and running:
```bash
docker ps
```
In the output you should be able to see the `gesture-control-app` container up and running. 

### How to use it?

Open the following link in your web browser to start controlling the robot using your gestures:
```
http://127.0.0.1:8888/
```

### Configuration Options

You can adjust the behavior of the application by specifying the following environment variables:

- `CAMERA_TYPE`: Choose between "webcam" (default) or "intel" (for Intel RealSense D435i model).
- `CONTROL_LOOP_RATE`: Set the control loop frequency in Hz for sending control commands to move the robot (default 50Hz ~ 20ms).

### Reference Documentation

- [MediaPipe](https://developers.google.com/mediapipe/solutions/vision/hand_landmarker)

