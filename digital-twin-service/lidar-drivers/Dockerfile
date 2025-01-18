# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=melodic
 
# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user with sudo privileges and set a password
RUN useradd -m lidar && \
    echo "lidar:lidar" | chpasswd && adduser lidar sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    git \
    lsof \
    ros-${ROS_DISTRO}-tf \
    python-rosinstall \
    python-rosinstall-generator \
    libeigen3-dev && \
    apt-get clean


# Switch to the newly created user for better security (avoid using root)
USER lidar
WORKDIR /home/lidar

# # Clone the necessary ROS packages into the catkin workspace
# RUN mkdir -p ~/catkin_ws/src && \
#     cd catkin_ws/src && \
#     git clone https://github.com/Slamtec/rplidar_ros.git --branch=master src/rplidar_ros && \
#     cd src/rplidar_ros && \
#     git checkout 7b011f142b489d448492b5e6a683293f1e482aaa

# # Clone the necessary ROS packages into the catkin workspace
# RUN mkdir -p ~/catkin_ws/src && \
#     cd catkin_ws/src && \
#     git clone https://github.com/robopeak/rplidar_ros.git

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/lidar/catkin_ws/src

# Build the catkin workspace
RUN cd catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make

# Add ROS environment setup to bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/lidar/scripts
COPY scripts/* /home/lidar/scripts/
WORKDIR /home/lidar/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh

COPY rules/* /etc/udev/rules.d/
RUN chmod +755 /etc/udev/rules.d/rplidar.rules

USER lidar
CMD "./wait-for-ros-nodes.sh"