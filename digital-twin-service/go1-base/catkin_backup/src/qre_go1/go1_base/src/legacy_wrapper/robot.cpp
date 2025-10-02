#include "go1_base/legacy_wrapper/robot.hpp"

using namespace qre;

go1_legged_msgs::HighState Robot::extractHighStateMessage() {
    go1_legged_msgs::HighState high_state;

    high_state.frameReserve = base_high_state.frameReserve;
    high_state.bandWidth = base_high_state.bandWidth;
    high_state.mode = base_high_state.mode;
    high_state.progress = base_high_state.progress;
    high_state.gaitType = base_high_state.gaitType;
    high_state.footRaiseHeight = base_high_state.footRaiseHeight;
    high_state.bodyHeight = base_high_state.bodyHeight;
    high_state.yawSpeed = base_high_state.yawSpeed;
    for(uint8_t i = 0; i<2; i++) {
        high_state.head[i] = base_high_state.head[i];
        high_state.SN[i] = base_high_state.SN[i];
        high_state.version[i] = base_high_state.version[i];
    }
    for(uint8_t i = 0; i<3; i++) {
        high_state.position[i] = base_high_state.position[i];
        high_state.velocity[i] = base_high_state.velocity[i];
        }
    for(uint8_t i = 0; i<4; i++) {
        high_state.footForce[i] = base_high_state.footForce[i];
        high_state.footForceEst[i] = base_high_state.footForceEst[i];
        high_state.rangeObstacle[i] = base_high_state.rangeObstacle[i];

        high_state.footPosition2Body[i].x = base_high_state.footPosition2Body[i].x;
        high_state.footPosition2Body[i].y = base_high_state.footPosition2Body[i].y;
        high_state.footPosition2Body[i].z = base_high_state.footPosition2Body[i].z;

        high_state.footSpeed2Body[i].x = base_high_state.footSpeed2Body[i].x;
        high_state.footSpeed2Body[i].y = base_high_state.footSpeed2Body[i].y;
        high_state.footSpeed2Body[i].z = base_high_state.footSpeed2Body[i].z;
        
        high_state.footForce[i] = base_high_state.footForce[i];
        high_state.footForceEst[i] = base_high_state.footForceEst[i];
    }
    for(uint8_t i = 0; i<40; i++) high_state.wirelessRemote[i] = base_high_state.wirelessRemote[i];
    high_state.reserve = base_high_state.reserve;
    high_state.crc = base_high_state.crc;
    return high_state;
}

go1_legged_msgs::MotorStateArray Robot::extractMotorStateMessage() {
    go1_legged_msgs::MotorStateArray motor_state_array;
    motor_state_array.motor_state.resize(20);
    for(uint8_t i = 0; i<20; i++) {
        motor_state_array.motor_state[i].mode = base_low_state.motorState[i].mode;        
        motor_state_array.motor_state[i].q    = base_low_state.motorState[i].q;        
        motor_state_array.motor_state[i].dq   = base_low_state.motorState[i].dq;        
        motor_state_array.motor_state[i].ddq  = base_low_state.motorState[i].ddq;        
        motor_state_array.motor_state[i].tauEst  = base_low_state.motorState[i].tauEst;        
        motor_state_array.motor_state[i].q_raw   = base_low_state.motorState[i].q_raw;        
        motor_state_array.motor_state[i].dq_raw  = base_low_state.motorState[i].dq_raw;        
        motor_state_array.motor_state[i].ddq_raw = base_low_state.motorState[i].ddq_raw;        
        motor_state_array.motor_state[i].temperature = base_low_state.motorState[i].temperature;        
        motor_state_array.motor_state[i].reserve[0]  = base_low_state.motorState[i].reserve[0];        
        motor_state_array.motor_state[i].reserve[1]  = base_low_state.motorState[i].reserve[1];        
    }
    return motor_state_array;
}

go1_legged_msgs::LowState Robot::extractLowStateMessage() {
    go1_legged_msgs::LowState low_state;
    low_state.frameReserve = base_low_state.frameReserve;
    low_state.levelFlag = base_low_state.levelFlag;
    low_state.bandWidth = base_low_state.bandWidth;
    for(uint8_t i = 0; i<2; i++) {
        low_state.head[i] = base_low_state.head[i];
        low_state.SN[i] = base_low_state.SN[i];
        low_state.version[i] = base_low_state.version[i];
    }
    for(uint8_t i = 0; i<4; i++) {
        low_state.footForce[i]    = base_low_state.footForce[i];
        low_state.footForceEst[i] = base_low_state.footForceEst[i];
    }
    for(uint8_t i = 0; i<40; i++) low_state.wirelessRemote[i] = base_low_state.wirelessRemote[i];
    low_state.reserve = base_low_state.reserve;
    low_state.crc = base_low_state.crc;
    low_state.tick = base_low_state.tick;
    return low_state;
}

sensor_msgs::Imu Robot::extractImuMessage() {
    sensor_msgs::Imu imu;

    imu.header.stamp    = ros::Time::now();
    imu.header.frame_id = "imu_link";

    imu.angular_velocity.x = base_high_state.imu.gyroscope[0];
    imu.angular_velocity.y = base_high_state.imu.gyroscope[1];
    imu.angular_velocity.z = base_high_state.imu.gyroscope[2];

    imu.linear_acceleration.x = base_high_state.imu.accelerometer[0];
    imu.linear_acceleration.y = base_high_state.imu.accelerometer[1];
    imu.linear_acceleration.z = base_high_state.imu.accelerometer[2];

    // version1
    // imu.orientation.x = base_high_state.imu.quaternion[1];
    // imu.orientation.y = base_high_state.imu.quaternion[2];
    // imu.orientation.z = base_high_state.imu.quaternion[3];
    // imu.orientation.w = base_high_state.imu.quaternion[0];  

    imu.orientation.x = base_high_state.imu.quaternion[0];
    imu.orientation.y = base_high_state.imu.quaternion[1];
    imu.orientation.z = base_high_state.imu.quaternion[2];
    imu.orientation.w = base_high_state.imu.quaternion[3];    
    return imu;
}

sensor_msgs::BatteryState Robot::extractBatteryStateMessage() {
    sensor_msgs::BatteryState battery_state;

    battery_state.header.stamp    = ros::Time::now();
    battery_state.header.frame_id = "battery_state";
    battery_state.percentage = base_high_state.bms.SOC/100.0;
    battery_state.charge  = base_high_state.bms.SOC;
    battery_state.current = base_high_state.bms.current;

    for (uint8_t i = 0; i < 30; i++) {
        battery_state.cell_voltage.push_back(base_high_state.bms.cell_vol[i]);
    }

    battery_state.location = "right_side_on_body";
    battery_state.serial_number = "GO1";
    battery_state.present = true;
    battery_state.power_supply_status     = battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health     = battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology = battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    return battery_state;
}

std::tuple<nav_msgs::Odometry, geometry_msgs::TransformStamped>
Robot::extractOdometryMessage() {
    nav_msgs::Odometry odometry;
    geometry_msgs::TransformStamped odometry_transform;

    // Single timestamp for consistency
    const ros::Time stamp = ros::Time::now();

    // --- Normalize/sanitize quaternion ---
    double qx = base_high_state.imu.quaternion[0];
    double qy = base_high_state.imu.quaternion[1];
    double qz = base_high_state.imu.quaternion[2];
    double qw = base_high_state.imu.quaternion[3];

    geometry_msgs::Quaternion quaternion;
    bool bad = !std::isfinite(qx) || !std::isfinite(qy) ||
               !std::isfinite(qz) || !std::isfinite(qw);
    if (bad) {
        quaternion.x = quaternion.y = quaternion.z = 0.0;
        quaternion.w = 1.0;
    } else {
        double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (n < 1e-6) {
            quaternion.x = quaternion.y = quaternion.z = 0.0;
            quaternion.w = 1.0;
        } else {
            // If vendor provides [w,x,y,z], swap before normalization.
            quaternion.x = qx / n;
            quaternion.y = qy / n;
            quaternion.z = qz / n;
            quaternion.w = qw / n;
        }
    }

    // --- Transform (odom -> base) ---
    odometry_transform.header.stamp = stamp;
    odometry_transform.header.frame_id = "odom";
    odometry_transform.child_frame_id  = "base";
    odometry_transform.transform.translation.x = base_high_state.position[0];
    odometry_transform.transform.translation.y = base_high_state.position[1];
    odometry_transform.transform.translation.z = 0.0;  // planar
    odometry_transform.transform.rotation = quaternion;

    // --- Odometry message ---
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id  = "base";

    odometry.pose.pose.position.x = base_high_state.position[0];
    odometry.pose.pose.position.y = base_high_state.position[1];
    odometry.pose.pose.position.z = base_high_state.bodyHeight;
    odometry.pose.pose.orientation = quaternion;

    odometry.twist.twist.linear.x  = base_high_state.velocity[0];
    odometry.twist.twist.linear.y  = base_high_state.velocity[1];
    odometry.twist.twist.linear.z  = 0.0;
    odometry.twist.twist.angular.x = 0.0;
    odometry.twist.twist.angular.y = 0.0;
    odometry.twist.twist.angular.z = base_high_state.bodyHeight;

    return {odometry, odometry_transform};
}


// std::tuple<nav_msgs::Odometry, geometry_msgs::TransformStamped> Robot::extractOdometryMessage() {
//     nav_msgs::Odometry odometry;
//     geometry_msgs::TransformStamped odometry_transform;
//     geometry_msgs::Quaternion quaternion;
//     int zeros[sizeof(base_high_state.imu.quaternion)];
//     if(memcmp(&base_high_state.imu.quaternion, zeros, sizeof(base_high_state.imu.quaternion))==0)
//         base_high_state.imu.quaternion[0] = 1.0;

//     quaternion.x = base_high_state.imu.quaternion[0];
//     quaternion.y = base_high_state.imu.quaternion[1];
//     quaternion.z = base_high_state.imu.quaternion[2];
//     quaternion.w = base_high_state.imu.quaternion[3];   

//     odometry_transform.header.stamp = ros::Time::now();
//     odometry_transform.header.frame_id = "odom";
//     odometry_transform.child_frame_id  = "base";
//     odometry_transform.transform.translation.x = base_high_state.position[0];
//     odometry_transform.transform.translation.y = base_high_state.position[1];
//     // odometry_transform.transform.translation.z = base_high_state.position[2];
//     odometry_transform.transform.translation.z = base_high_state.bodyHeight;
//     odometry_transform.transform.rotation = quaternion;

//     odometry.header.stamp = ros::Time::now();
//     odometry.header.frame_id = "odom";
//     odometry.pose.pose.position.x = base_high_state.position[0];
//     odometry.pose.pose.position.y = base_high_state.position[1];
//     // odometry.pose.pose.position.z = base_high_state.position[2];
//     odometry.pose.pose.position.z = base_high_state.bodyHeight;
//     odometry.pose.pose.orientation = quaternion;
//     odometry.child_frame_id = "base";
//     odometry.twist.twist.linear.x  = base_high_state.velocity[0];
//     odometry.twist.twist.linear.y  = base_high_state.velocity[1];
//     odometry.twist.twist.angular.z = base_high_state.velocity[2];
//     return {odometry, odometry_transform};
// }
