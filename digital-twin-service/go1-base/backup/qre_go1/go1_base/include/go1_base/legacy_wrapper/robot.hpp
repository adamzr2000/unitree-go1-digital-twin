#ifndef _ROBOT_H
#define _ROBOT_H 

#include <ros/ros.h>
#include <tuple>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "go1_legged_msgs/HighState.h"
#include "go1_legged_msgs/LowState.h"
#include "go1_legged_msgs/MotorStateArray.h"
#include "go1_legged_msgs/BmsState.h"


namespace qre {
    // A wrapper class from UT SDK to ROS
    class Robot {
        public:
            UT::HighCmd   robot_high_cmd;
            UT::HighState base_high_state;
            UT::LowCmd    robot_low_cmd;
            UT::LowState  base_low_state;
            uint8_t mode = 2;
            uint8_t gait_type = 2;
            uint8_t speed_level = 0;
            float foot_raise_height;
            float body_height;
            std::array<float, 2> position;
            std::array<float, 3> euler;
            go1_legged_msgs::HighState extractHighStateMessage();
            go1_legged_msgs::LowState  extractLowStateMessage();
            go1_legged_msgs::MotorStateArray extractMotorStateMessage();
            sensor_msgs::Imu extractImuMessage();
            sensor_msgs::BatteryState extractBatteryStateMessage();
            std::tuple<nav_msgs::Odometry, geometry_msgs::TransformStamped> extractOdometryMessage();
    };
}

#endif 