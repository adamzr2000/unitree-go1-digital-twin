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

# Pre-configure debconf for keyboard layout
RUN echo 'keyboard-configuration keyboard-configuration/layout select Spanish' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/layoutcode select es' | debconf-set-selections


# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-catkin-tools \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    ninja-build stow 

# Switch to the newly created user for better security (avoid using root)
USER go1
WORKDIR /home/go1

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p ~/carto_ws/src && \
    cd carto_ws && \
    vcs import src --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall && \
    sed -i -e "s%<depend>libabsl-dev</depend>%<\!--<depend>libabsl-dev</depend>-->%g" /home/go1/carto_ws/src/cartographer/package.xml

USER root

# Update rosdep
RUN rosdep update

RUN cd /home/go1/carto_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    src/cartographer/scripts/install_abseil.sh

USER go1 

# Build the catkin workspace
RUN cd ~/carto_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make_isolated --install --use-ninja

# Add ROS environment setup to bashrc 
RUN echo "source ~/carto_ws/install_isolated/setup.bash" >> ~/.bashrc


# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/go1/catkin_ws/src

# Switch back to root to change file permissions
USER root

RUN cd /home/go1/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y 

RUN cd /home/go1/catkin_ws/src/qre_go1/ && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ./go1_install.bash

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
# CMD "./slam_launch.sh"
