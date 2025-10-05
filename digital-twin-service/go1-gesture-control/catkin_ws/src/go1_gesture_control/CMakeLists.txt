cmake_minimum_required(VERSION 2.8.3)
project(go1_gesture_control)

set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(circle_walk scripts/circle_walk.cpp)
target_link_libraries(circle_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(circle_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gesture_control scripts/gesture_control.cpp)
target_link_libraries(gesture_control ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(gesture_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})