FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-lc"]

RUN apt-get update
RUN useradd -ms /bin/bash vizanti
RUN mkdir -p /home/vizanti/catkin_ws/src
WORKDIR "/home/vizanti/catkin_ws"

COPY . src/vizanti/

RUN rosdep update --include-eol-distros && \
    rosdep install -i --rosdistro noetic --from-path src/vizanti -y

RUN source /opt/ros/noetic/setup.bash && catkin_make

USER vizanti
CMD source /home/vizanti/catkin_ws/devel/setup.bash && roslaunch vizanti server.launch
