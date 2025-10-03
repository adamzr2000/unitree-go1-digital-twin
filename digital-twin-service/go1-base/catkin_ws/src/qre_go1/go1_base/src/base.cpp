#include "go1_base/base.hpp"

using namespace qre;
Base::Base(UT::HighCmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh): robot_safety(UT::LeggedType::Go1) {
    nh_ = nh;
    p_nh_ = p_nh;
    level = UT::HIGHLEVEL;
    robot_high_cmd.mode            = 2;
    robot_high_cmd.gaitType        = 1;
    robot_high_cmd.velocity        = {0.0};
    robot_high_cmd.position        = {0.0};
    robot_high_cmd.yawSpeed        = 0.0;
    robot_high_cmd.euler           = {0.0};
    robot_high_cmd.bodyHeight      = 0.0;
    robot_high_cmd.footRaiseHeight = 0.0;
    odom_publisher  = p_nh_->advertise<nav_msgs::Odometry>("odom", 1);
    cmd_subscriber  = p_nh_->subscribe("cmd_vel", 1, &Base::cmdVelCallback, this);
    state_publisher = p_nh_->advertise<go1_legged_msgs::HighState>("state", 1);
    imu_publisher   = p_nh_->advertise<sensor_msgs::Imu>("imu/data", 1);
    motor_state_publisher   = nh_->advertise<go1_legged_msgs::MotorStateArray>("motor_states", 1);
    battery_state_publisher = nh_->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    joint_state_publisher   = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);
    set_mode = nh_->advertiseService("set_mode", &Base::setModeCallback, this);
    std::string ip_string;    
    p_nh_->param<std::string>("target_ip", ip_string, "192.168.123.161");
    strcpy(udp_ip, ip_string.c_str());
    int target_port, local_port;
    p_nh_->param<int>("target_port", target_port, 8082);
    p_nh_->param<int>("local_port",  local_port,  8090);
    std::cout << "Initializing UDP with ip: " << ip_string << ", local port: " << local_port << ", target port: " << target_port << std::endl; 
    comm_bus = new UT::UDP(level, local_port, const_cast<const char*>(udp_ip), target_port);
    comm_bus->InitCmdData(robot_high_cmd);
}

Base::Base(UT::LowCmd  cmd, UT::LowState  state, ros::NodeHandle *nh, ros::NodeHandle *p_nh): robot_safety(UT::LeggedType::Go1) {
    nh_   = nh;
    p_nh_ = p_nh;
    level = UT::LOWLEVEL;
    cmd_subscriber  = p_nh_->subscribe("joint_cmd", 1, &Base::jointCommandCallback, this);
    state_publisher = p_nh_->advertise<go1_legged_msgs::LowState>("state", 1);
    imu_publisher   = p_nh_->advertise<sensor_msgs::Imu>("imu/data", 1);
    joint_state_publisher   = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);
    battery_state_publisher = nh_->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    motor_state_publisher   = p_nh_->advertise<go1_legged_msgs::MotorStateArray>("motor_states", 1);
    set_control = p_nh_->advertiseService("set_control", &Base::setControlCallback, this);
    for(int i = 0; i<12; i++) {
        robot_low_cmd.motorCmd[i].mode = 0x0A;  
        robot_low_cmd.motorCmd[i].q    = UT::PosStopF;        
        robot_low_cmd.motorCmd[i].Kp   = 0;
        robot_low_cmd.motorCmd[i].dq   = UT::VelStopF;
        robot_low_cmd.motorCmd[i].Kd   = 0;
        robot_low_cmd.motorCmd[i].tau  = 0;
    }
    ros::param::set("~/control_type", "Position");
    std::string ip_string;    
    p_nh_->param<std::string>("target_ip", ip_string, "192.168.123.10");
    strcpy(udp_ip, ip_string.c_str());
    int target_port, local_port;
    p_nh_->param<int>("target_port", target_port, 8007);
    p_nh_->param<int>("local_port",  local_port,  8090);
    std::cout << "Initializing UDP with ip: " << ip_string << ", local port: " << local_port << ", target port: " << target_port << std::endl; 
    comm_bus = new UT::UDP(level, local_port, const_cast<const char*>(udp_ip), target_port);
    comm_bus->InitCmdData(robot_low_cmd);
}

void Base::packetReceive() {
    comm_bus->Recv();
}

void Base::packetSend() {  
    comm_bus->Send();
}

void Base::HighLevelControl() {
    comm_bus->GetRecv(base_high_state);
    comm_bus->SetSend(robot_high_cmd);
}

void Base::LowLevelControl() {
    comm_bus->GetRecv(base_low_state);
    comm_bus->SetSend(robot_low_cmd);
}

void Base::publishStateMessages() {
    if (level == UT::HIGHLEVEL) {
        auto state_msg = extractHighStateMessage();
        auto joint_state_msg = getJointStates();
        auto [odom, odom_transform] = extractOdometryMessage();
        // odom_broadcaster.sendTransform(odom_transform);
        odom_publisher.publish(odom);
        joint_state_publisher.publish(joint_state_msg);
        state_publisher.publish(state_msg);
    }
    else if (level == UT::LOWLEVEL) {
        auto state_msg   = extractLowStateMessage();
        auto joint_angle = extractJointAngles();
        state_publisher.publish(state_msg);
        joint_state_publisher.publish(joint_angle);
    }
    auto battery_state_msg = extractBatteryStateMessage();
    auto motor_state_msg   = extractMotorStateMessage();
    auto imu_msg = extractImuMessage();
    motor_state_publisher.publish(motor_state_msg);
    imu_publisher.publish(imu_msg);
}

bool Base::setModeCallback(go1_legged_msgs::SetMode::Request &req, go1_legged_msgs::SetMode::Response &res) {
    robot_high_cmd.mode = req.mode;
    robot_high_cmd.gaitType = req.gait_type;
    res.success = true;
    return true;
}

bool Base::setControlCallback(go1_legged_msgs::SetControl::Request &req, go1_legged_msgs::SetControl::Response &res) {
    if(std::any_of(std::begin(modes), std::end(modes), [=](std::string mode) {return mode == req.control;})) {    
        low_level_control_type = req.control;
        ros::param::set("~/control_type", low_level_control_type);
        res.success = true;
        res.message = "Low level control changed to: " + low_level_control_type;
        return true;
        // }
    }
    else {
        res.success = false;
                res.message = "Select low level control mode to Position, Velocity, Torque or Full";
        return false; 
    }   
}

void Base::cmdVelCallback(geometry_msgs::Twist msg) {
    robot_high_cmd.velocity = {0.0};
    robot_high_cmd.yawSpeed = 0.0;  
  
    if (msg.twist.linear.x && msg.twist.linear.y && msg.twist.angular.z) {
        robot_high_cmd.velocity[0] = msg.twist.linear.x;
        robot_high_cmd.velocity[1] = msg.twist.linear.y;
        robot_high_cmd.yawSpeed = msg.twist.angular.z;
    }
    else if (msg.twist.linear.x && msg.twist.linear.y) {
        robot_high_cmd.velocity[0] = msg.twist.linear.x;
        robot_high_cmd.velocity[1] = msg.twist.linear.y;
    }
    else if (msg.twist.linear.x && msg.twist.angular.z) {
        robot_high_cmd.velocity[0] = msg.twist.linear.x;
        robot_high_cmd.yawSpeed = msg.twist.angular.z;
    }
    else if (msg.twist.linear.x) {
        robot_high_cmd.velocity[0] = msg.twist.linear.x;        
    }
    else if (msg.twist.linear.y) {
         robot_high_cmd.velocity[1] = msg.twist.linear.y;        
    }
    else if (msg.twist.angular.z) {
        robot_high_cmd.yawSpeed = msg.twist.angular.z;
    }
    else {
        robot_high_cmd.velocity = {0.0};
        robot_high_cmd.yawSpeed = 0.0;        
    } 
}

void Base::jointCommandCallback(go1_legged_msgs::JointCmd msg) {
        if (low_level_control_type != "Torque" && low_level_control_type != "Full") {
        robot_low_cmd.motorCmd[UT::FR_0].tau = -0.65f;
        robot_low_cmd.motorCmd[UT::FL_0].tau = +0.65f;
        robot_low_cmd.motorCmd[UT::RR_0].tau = -0.65f;
        robot_low_cmd.motorCmd[UT::RL_0].tau = +0.65f;
    }
    if(low_level_control_type == "Position") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q  = msg.q[i]; 
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i]; 
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i]; 
        }
    }
    else if(low_level_control_type == "Velocity") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q  = UT::PosStopF;          
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i];
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i];
            robot_low_cmd.motorCmd[i].dq = msg.dq[i];
        }
    }
    else if(low_level_control_type == "Torque") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q   = UT::PosStopF; 
            robot_low_cmd.motorCmd[i].dq  = UT::VelStopF; 
            robot_low_cmd.motorCmd[i].tau = msg.tau[i];
        }
    }   
    else if (low_level_control_type == "Full"){
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q  = msg.q[i];
            robot_low_cmd.motorCmd[i].dq  = msg.dq[i];
            robot_low_cmd.motorCmd[i].tau  = msg.tau[i];
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i];
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i];
        }
    }
  
}

void qre::heartbeatMonitor() {  
    std::string ip_address;    
    ros::param::param<std::string>("target_ip", ip_address, "192.168.123.161");
    ros::Rate loop_rate(10);
    uint16_t count(0);
    while (ros::ok()) {
        std::string command = std::string("ping -c1 -w1 -s1 ") + ip_address + std::string("  > /dev/null 2>&1");
        int status = system(command.c_str());
        if (status!=0) {
            ROS_WARN("No connection to robot found.");
            ros::shutdown();
            count ++;
            if (count > 10) {
                ROS_ERROR("\nShutting down controller.");
                ros::shutdown();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "\nStopping heartbeat monitor" << std::endl;
}
