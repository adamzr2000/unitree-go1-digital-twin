# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=melodic
 
# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user with sudo privileges and set a password
RUN useradd -m d435i && \
    echo "d435i:d435i" | chpasswd && adduser d435i sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    apt-transport-https

COPY rules/99-realsense-libusb.rules /etc/udev/rules.d/

RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo xenial main"
RUN apt-get update -qq
RUN apt-get install librealsense2-dkms --allow-unauthenticated -y
RUN apt-get install librealsense2-dev --allow-unauthenticated -y

RUN apt -q -qq update && \
  DEBIAN_FRONTEND=noninteractive apt install -y --allow-unauthenticated \
  python-rosinstall \
  python-catkin-tools \
  ros-${ROS_DISTRO}-jsk-tools \
  ros-${ROS_DISTRO}-rgbd-launch \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-image-transport \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

RUN rosdep update

# Switch to the newly created user for better security (avoid using root)
USER d435i
WORKDIR /home/d435i

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src && cd catkin_ws/src && \
  git clone --branch ros1-legacy --depth 1 https://github.com/IntelRealSense/realsense-ros.git && \
  git clone --depth 1 https://github.com/pal-robotics/ddynamic_reconfigure

# Build the catkin workspace
RUN cd /home/d435i/catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make

# Add ROS environment setup to bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/d435i/scripts
COPY scripts/* /home/d435i/scripts/
WORKDIR /home/d435i/scripts/

USER root
RUN chmod +755 wait-for-ros-nodes.sh

USER d435i
CMD "./wait-for-ros-nodes.sh"