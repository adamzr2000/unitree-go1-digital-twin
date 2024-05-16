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
    screen \
    iperf3 \
    iproute2 \
    iputils-ping \
    net-tools \
    ethtool \
    expect \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN pip3 install influxdb-client

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/go1/app
COPY app/ /home/go1/app/
WORKDIR /home/go1/app/

USER root

# Make all scripts executable
RUN chmod +755 *.sh

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

USER go1

CMD "./start_app.sh"

