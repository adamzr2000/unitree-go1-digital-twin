# Specify the ROS distribution as a build argument
ARG ROS_DISTRO=noetic
 
# Use an official ROS base image 
FROM ros:${ROS_DISTRO}

# Label to indicate the maintainer of this Dockerfile
LABEL maintainer="azahir@pa.uc3m.es"

# Setup the working directory in the container
WORKDIR /root

# Create a new user with sudo privileges and set a password
RUN useradd -m phidgets-imu && \
    echo "phidgets-imu:phidgets-imu" | chpasswd && adduser phidgets-imu sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash", "-c" ]

# Install essential tools and clean up after to reduce image size
RUN apt-get update && apt-get install -y \
    git-core \
    lsof \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    ros-$ROS_DISTRO-imu-transformer \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    && rm -rf /var/lib/apt/lists/*


# Switch to the newly created user for better security (avoid using root)
USER phidgets-imu
WORKDIR /home/phidgets-imu

# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src catkin_ws/src

# Install dependencies using rosdep
# RUN cd catkin_ws/src && source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep init && rosdep install phidgets_drivers

# Build the catkin workspace
RUN cd catkin_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make

# Add ROS environment setup to bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /home/phidgets-imu/scripts/
COPY scripts/ .

USER root

COPY rules/* /etc/udev/rules.d/
RUN chmod +755 /etc/udev/rules.d/99-phidgets.rules

RUN chmod +755 wait-for-ros-nodes.sh

USER phidgets-imu

CMD "./wait-for-ros-nodes.sh"