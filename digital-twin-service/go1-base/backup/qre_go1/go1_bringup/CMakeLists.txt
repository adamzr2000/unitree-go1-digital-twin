cmake_minimum_required(VERSION 3.0.2)
project(go1_bringup)
find_package(catkin REQUIRED COMPONENTS
  go1_base
  go1_control
  go1_description
  robot_upstart
)