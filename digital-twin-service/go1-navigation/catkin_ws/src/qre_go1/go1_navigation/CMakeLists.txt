cmake_minimum_required(VERSION 3.0.2)
project(go1_navigation)

find_package(catkin REQUIRED COMPONENTS
  teb_local_planner
  map_server
  go1_control
  move_base
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)