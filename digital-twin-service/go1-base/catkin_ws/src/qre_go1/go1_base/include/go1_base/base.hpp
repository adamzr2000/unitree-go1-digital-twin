#ifndef _BASE_H
#define _BASE_H 

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
// #include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "legacy_wrapper/quadruped.hpp"
#include "go1_legged_msgs/SetMode.h"
#include "go1_legged_msgs/SetControl.h"
#include "go1_legged_msgs/JointCmd.h"



namespace qre{
    void heartbeatMonitor();
    class Base : public Quadruped {
        ros::NodeHandle *nh_, *p_nh_;

        ros::Subscriber cmd_subscriber;
        ros::Publisher joint_state_publisher;
        ros::Publisher state_publisher;
        ros::Publisher imu_publisher;
        ros::Publisher battery_state_publisher;
        ros::Publisher motor_state_publisher;
        ros::Publisher odom_publisher;
        // tf::TransformBroadcaster odom_broadcaster;
        ros::ServiceServer set_mode;    
        ros::ServiceServer set_control;    

        UT::Safety robot_safety;
        UT::UDP* comm_bus;
        uint16_t level;
                std::string modes[4] = {"Position", "Velocity", "Torque", "Full"};
        std::string low_level_control_type = "Position";
        char udp_ip[16];        

    public:
        float dt = 0.002;     // 0.001~0.01
        Base(UT::HighCmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
        Base(UT::LowCmd  cmd, UT::LowState  state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
        void HighLevelControl();
        void LowLevelControl();
        void packetReceive();
        void packetSend();
        // void cmdVelCallback(geometry_msgs::TwistStamped msg);
        void cmdVelCallback(geometry_msgs::Twist msg);
        void jointCommandCallback(go1_legged_msgs::JointCmd msg);
        void publishStateMessages();
        bool setModeCallback(go1_legged_msgs::SetMode::Request &req, go1_legged_msgs::SetMode::Response &res);
        bool setControlCallback(go1_legged_msgs::SetControl::Request &req, go1_legged_msgs::SetControl::Response &res);
    };
}

#endif 
