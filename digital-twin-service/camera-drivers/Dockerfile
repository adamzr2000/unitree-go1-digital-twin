# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=melodic
 
# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user with sudo privileges and set a password
RUN useradd -m astra && \
    echo "astra:astra" | chpasswd && adduser astra sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
  libgflags-dev \
  ros-${ROS_DISTRO}-image-geometry \
  ros-${ROS_DISTRO}-camera-info-manager\
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-publisher \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-ros \
  libgoogle-glog-dev \
  libusb-1.0-0-dev \
  libeigen3-dev \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

# Clone the libuvc repository
RUN git clone https://github.com/libuvc/libuvc.git

# Build and install libuvc
RUN cd libuvc && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Run ldconfig to update the shared library cache
RUN ldconfig

# Switch to the newly created user for better security (avoid using root)
USER astra
WORKDIR /home/astra

RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
    git clone https://github.com/orbbec/ros_astra_camera

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make 

# Add ROS environment setup to bashrc of user 'go1'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/astra/scripts
COPY scripts/* /home/astra/scripts/
WORKDIR /home/astra/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh

COPY rules/* /etc/udev/rules.d/
RUN chmod +755 /etc/udev/rules.d/56-orbbec-usb.rules

USER astra
# CMD "./wait-for-ros-nodes.sh"