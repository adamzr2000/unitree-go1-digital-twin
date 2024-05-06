# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=noetic

# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user 'go1' with sudo privileges and set a password
RUN useradd -m go1 && \
    echo "go1:go1" | chpasswd && adduser go1 sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    git \
    wget \
    unzip \
    python3-pip \
    screen \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-turtlesim \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtk2.0-dev \
 && rm -rf /var/lib/apt/lists/* \
 && apt-get clean

RUN pip3 install --timeout=1000 mediapipe numpy==1.20 pyrealsense2 Flask \
 && rm -rf /root/.cache/pip

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/go1/catkin_ws/src

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make 

# Add ROS environment setup to bashrc of user 'go1'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Switch back to root to change file permissions
USER root

# Change permissions to make scripts executable
RUN chmod a+x /home/go1/catkin_ws/src/unitree_ros_to_real/unitree_legged_real/scripts/*

# Switch back to user go1
USER go1

RUN mkdir -p /home/go1/app
COPY app/ /home/go1/app/
WORKDIR /home/go1/app/

USER root

# Make all scripts executable
RUN chmod +755 *.sh

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

USER go1

# Start command to run inside the Docker container
CMD "./start_app.sh"