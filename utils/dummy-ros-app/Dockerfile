# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=noetic

# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Set up user with sudo access
WORKDIR /root

# Create a new user 'robot' with sudo privileges and set a password
RUN useradd -m robot && \
    echo "robot:robot" | chpasswd && \
    adduser robot sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools
RUN apt-get update && apt-get install -y \
    git \
    wget \
    iproute2 \
    iputils-ping \
    traceroute \
    tshark \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Switch to the newly created user for better security (avoid using root)
USER robot
WORKDIR /home/robot

RUN mkdir scripts 
WORKDIR /home/robot/scripts

COPY scripts/* /home/robot/scripts/

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

CMD "./entrypoint.sh"