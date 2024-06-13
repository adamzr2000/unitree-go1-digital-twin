# Specify the ROS distribution as a build argument
FROM tiryoh/ros-desktop-vnc:noetic

# Define maintainer
LABEL maintainer="azahir@pa.uc3m.es"

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash" , "-c" ]

# Install necessary packages and cleaning up the apt cache to reduce the image size
RUN apt-get update && apt-get install -y \
    git \
    wget \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-tf-tree \
    ros-noetic-xacro \
    ros-noetic-turtlesim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y \
#     git \
#     wget \
#     ros-noetic-cartographer-rviz \
#     ros-noetic-rqt-graph \
#     ros-noetic-rqt-tf-tree \
#     ros-noetic-xacro \
#     ros-noetic-turtlesim 


# Clone the necessary ROS packages into the catkin workspace
RUN mkdir -p catkin_ws/src 
COPY catkin_ws/src /home/ubuntu/catkin_ws/src

# Build the catkin workspace
RUN cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make 

# Add ROS environment setup to bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/ubuntu/scripts
COPY scripts/* /home/ubuntu/scripts/
RUN chmod +755 /home/ubuntu/scripts/*.sh

RUN sed -i '/ROS_IP/d' /home/ubuntu/.bashrc
RUN sed -i '/ROS_MASTER_URI/d' /home/ubuntu/.bashrc

# Set the working directory to where your scripts are
WORKDIR /home/ubuntu/scripts/

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

# Use ENTRYPOINT to specify the executable script
# ENTRYPOINT ["./wait-for-ros-nodes.sh"]