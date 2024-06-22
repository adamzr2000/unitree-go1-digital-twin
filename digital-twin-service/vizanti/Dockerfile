# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=noetic

# Use an official ROS base image
FROM ros:${ROS_DISTRO}

LABEL maintainer="azahir@pa.uc3m.es"

# Set up user with sudo access
WORKDIR /root

# Create a new user 'go1' with sudo privileges and set a password
RUN useradd -m go1 && \
    echo "go1:go1" | chpasswd && \
    adduser go1 sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash" , "-c" ]

# Install necessary packages and cleaning up the apt cache to reduce the image size
RUN apt-get update && apt-get install -y \
    git \
    wget \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-move-base-msgs \
    ros-${ROS_DISTRO}-tf \
    libeigen3-dev \
    python3-flask

# Switch to the created user
USER go1
WORKDIR /home/go1

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/go1/catkin_ws/src

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src  && catkin_make 

# Add ROS environment setup to bashrc of user 'go1'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/go1/scripts
COPY scripts/* /home/go1/scripts/
WORKDIR /home/go1/scripts/

USER root

# Make all scripts executable
RUN chmod +755 *.sh

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

USER go1
CMD "./wait-for-ros-nodes.sh"


