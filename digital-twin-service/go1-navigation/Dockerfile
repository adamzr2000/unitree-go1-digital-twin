ARG ROS_DISTRO=noetic
FROM osrf/ros:${ROS_DISTRO}-desktop-full

LABEL maintainer="azahir@pa.uc3m.es"

ARG CARTOGRAPHER_VERSION=master
ARG CARTOGRAPHER_SHA=LATEST
ENV DEBIAN_FRONTEND=noninteractive

# Create user and working directory
RUN useradd -m go1 && echo "go1:go1" | chpasswd && adduser go1 sudo
USER go1
WORKDIR /home/go1

# Install base dependencies
USER root
RUN apt-get update && apt-get install -y \
    git wget libcairo2-dev

USER go1

# Prepare workspace and clone sources
COPY cartographer_ros/cartographer_ros.rosinstall cartographer_ros/scripts/prepare_catkin_workspace.sh ./scripts/
RUN CARTOGRAPHER_SHA=$CARTOGRAPHER_SHA \
    CARTOGRAPHER_VERSION=$CARTOGRAPHER_VERSION \
    ./scripts/prepare_catkin_workspace.sh && \
    sed -i 's%<depend>libabsl-dev</depend>%<!-- <depend>libabsl-dev</depend> -->%' carto_ws/src/cartographer/package.xml

# Copy updated package.xml files before rosdep
COPY cartographer_ros/cartographer_ros/package.xml carto_ws/src/cartographer_ros/cartographer_ros/
COPY cartographer_ros/cartographer_ros_msgs/package.xml carto_ws/src/cartographer_ros/cartographer_ros_msgs/
COPY cartographer_ros/cartographer_rviz/package.xml carto_ws/src/cartographer_ros/cartographer_rviz/

# Install ROS deps
COPY cartographer_ros/scripts/install_debs.sh ./scripts/
USER root
RUN ./scripts/install_debs.sh

# Install Abseil manually (as per official instructions)
RUN carto_ws/src/cartographer/scripts/install_abseil.sh

USER go1

# Copy build scripts
COPY cartographer_ros/scripts/install.sh \
     cartographer_ros/scripts/catkin_test_results.sh \
     cartographer_ros/scripts/ros_entrypoint.sh ./scripts/

# Build cartographer core
RUN ./scripts/install.sh --pkg cartographer --make-args "-j2" && \
    ./scripts/install.sh --pkg cartographer --make-args "test -j2"

# Build and test cartographer_ros_msgs
COPY cartographer_ros/cartographer_ros_msgs carto_ws/src/cartographer_ros/cartographer_ros_msgs/
RUN ./scripts/install.sh --pkg cartographer_ros_msgs && \
    ./scripts/install.sh --pkg cartographer_ros_msgs --catkin-make-args run_tests && \
    ./scripts/catkin_test_results.sh build_isolated/cartographer_ros_msgs

# Build and test cartographer_ros
COPY cartographer_ros/cartographer_ros carto_ws/src/cartographer_ros/cartographer_ros/
RUN ./scripts/install.sh --pkg cartographer_ros --make-args "-j2" && \
    ./scripts/install.sh --pkg cartographer_ros --catkin-make-args "run_tests -j2" && \
    ./scripts/catkin_test_results.sh build_isolated/cartographer_ros

# Build and test cartographer_rviz
COPY cartographer_ros/cartographer_rviz carto_ws/src/cartographer_ros/cartographer_rviz/

RUN cd carto_ws && ./install_isolated/env.sh ninja -j2 -C build_isolated/cartographer_ros -v

RUN ./scripts/install.sh --pkg cartographer_rviz && \
    ./scripts/install.sh --pkg cartographer_rviz --catkin-make-args run_tests && \
    ./scripts/catkin_test_results.sh build_isolated/cartographer_rviz

# Environment setup on container start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ~/carto_ws/install_isolated/setup.bash" >> ~/.bashrc


# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/go1/catkin_ws/src

# Switch back to root to change file permissions
USER root

RUN cd /home/go1/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y 

RUN cd /home/go1/catkin_ws/src/qre_go1/ && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ./go1_install.bash

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

USER go1

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/go1/carto_ws/install_isolated/setup.bash && catkin build -v

# Add ROS environment setup to bashrc of user 'go1'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/go1/scripts
COPY scripts/* /home/go1/scripts/
WORKDIR /home/go1/scripts/

USER root

# Make all scripts executable
RUN chmod +755 *.sh

USER go1
CMD "./wait-for-ros-nodes.sh"
