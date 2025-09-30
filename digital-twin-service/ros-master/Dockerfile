# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=noetic

# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

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

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    git \
    wget \
    screen \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy \
    -v ./scripts:/home/go1/scripts \
    && rm -rf /var/lib/apt/lists/*

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD bash -c "source /opt/ros/noetic/setup.bash && roscore"
