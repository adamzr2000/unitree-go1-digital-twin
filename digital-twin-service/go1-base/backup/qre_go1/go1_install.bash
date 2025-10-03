#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 1)$1$(tput sgr0)"
}

function install_binary_packages () {
    color_echo "Installing build packages."
    sudo apt-get install libpcl-dev build-essential \
                         cmake \
                         libglfw3-dev \
                         libglew-dev \
                         libeigen3-dev \
                         libjsoncpp-dev \
                         libtclap-dev \
                         libboost-all-dev -y
    color_echo "Installing ROS packages."
    sudo apt-get install ros-$ROS_DISTRO-hector-sensors-description \
                         ros-$ROS_DISTRO-robot-upstart \
                         ros-$ROS_DISTRO-teleop-twist-keyboard \
                         ros-$ROS_DISTRO-teleop-twist-joy \
                         ros-$ROS_DISTRO-geodesy \
                         ros-$ROS_DISTRO-nmea-msgs \
                         ros-$ROS_DISTRO-libg2o \
                         ros-$ROS_DISTRO-robot-localization \
                         ros-$ROS_DISTRO-roslaunch \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-std-msgs \
                         ros-$ROS_DISTRO-smach \
                         ros-$ROS_DISTRO-actionlib \
                         ros-$ROS_DISTRO-geometry-msgs \
                         ros-$ROS_DISTRO-move-base-msgs \
                         ros-$ROS_DISTRO-ecl-threads \
                         ros-$ROS_DISTRO-rospy \
                         ros-$ROS_DISTRO-roscpp \
                         ros-$ROS_DISTRO-controller-interface \
                         ros-$ROS_DISTRO-joint-state-controller \
                         ros-$ROS_DISTRO-joint-state-publisher \
                         ros-$ROS_DISTRO-effort-controllers \
                         ros-$ROS_DISTRO-tile-map \
                         ros-$ROS_DISTRO-multires-image \
                         ros-$ROS_DISTRO-xacro \
                         ros-$ROS_DISTRO-joint-trajectory-controller -y
}

RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 
                                                                                          
echo -e "${DGREEN}------------------------------------------------------------------------------------------------------"
echo -e " _______           _______  ______   _______           _______  _______  ______     ______   _______  "
echo -e "(  ___  )|\     /|(  ___  )(  __  \ (  ____ )|\     /|(  ____ )(  ____ \(  __  \   (  __  \ (  ____ \ "
echo -e "| (   ) || )   ( || (   ) || (  \  )| (    )|| )   ( || (    )|| (    \/| (  \  )  | (  \  )| (    \/ "
echo -e "| |   | || |   | || (___) || |   ) || (____)|| |   | || (____)|| (__    | |   ) |  | |   ) || (__     "
echo -e "| |   | || |   | ||  ___  || |   | ||     __)| |   | ||  _____)|  __)   | |   | |  | |   | ||  __)    "
echo -e "| | /\| || |   | || (   ) || |   ) || (\ (   | |   | || (      | (      | |   ) |  | |   ) || (       "
echo -e "| (_\ \ || (___) || )   ( || (__/  )| ) \ \__| (___) || )      | (____/\| (__/  )_ | (__/  )| (____/\ "
echo -e "(____\/_)(_______)|/     \|(______/ |/   \__/(_______)|/       (_______/(______/(_)(______/ (_______/ "
echo -e ""                                                                                                                                   
echo -e "------------------------------------------------------------------------------------------------------"
echo -e "Installing Required Libraries and ROS dependencies! "                                                                                                                                         
echo -e "------------------------------------------------------------------------------------------------------${NC}"

# Binary packages installation
install_binary_packages