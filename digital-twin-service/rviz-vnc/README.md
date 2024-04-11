# ROS RViz VNC. 

## What is it?


## What does this container do?


## Run it?

### Dependencies:

The ROS RViz VNC depends on:
  - ros-master (tutorial [here](../ros-master/))

### First you will need to build the container. 

In order to do that, run the following script:
```bash
./build.sh
```

### This will build the `go1-rviz-vnc` docker image. 

Verify that the image is present by running:
```bash
docker image ls
```

### Docker run example
In this folder we also provide a docker run example. 

To run the ROS RViz VNC:
```bash
./run_example.sh
```

Verify that the container is up and running:
```bash
docker ps
```

In the output you should be able to see the `rviz-vnc` container up and running.

### How to use it?

Open the following link in your web browser to access the VNC ctonainer with GUI:
```
http://127.0.0.1:6080/
```

Open a terminal and run the following command to launch RViZ:
```bash
rosrun rviz rviz
```


