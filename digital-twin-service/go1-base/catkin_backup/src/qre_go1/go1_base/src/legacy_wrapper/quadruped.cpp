#include "go1_base/legacy_wrapper/quadruped.hpp"

Quadruped::Quadruped() { 
  Eigen::Matrix<double, 4, 3> robot_config_matrix;
  robot_config_matrix << trunk_length, -trunk_width, 0.0,
                         trunk_length,  trunk_width, 0.0,
                        -trunk_length, -trunk_width, 0.0,
                        -trunk_length,  trunk_width, 0.0;    
  for(uint8_t i=0; i<4; i++) {
      auto rotation_matrix = rotxyz(M_PI/2, -M_PI/2,0); // Correction of orientation
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
      transformation_matrix.block<3,3>(0,0) = rotation_matrix;
      transformation_matrix.block<3,1>(0,3) = robot_config_matrix.row(i);   
      go1_config_matrix[i] = transformation_matrix;
    }    
}

Quadruped::~Quadruped() {}

Eigen::Matrix<double, 4, 1>* Quadruped::footTransformsFromPositions() {
  static Eigen::Matrix<double, 4, 1> foot_transform[4];
  for (uint8_t i = 0; i<4; i++) {
    Eigen::Matrix<double, 4, 1> foot_position = Eigen::Matrix <double, 4, 1>::Zero();
    foot_position << base_high_state.footPosition2Body[i].x, base_high_state.footPosition2Body[i].y, base_high_state.footPosition2Body[i].z, 1.0;
    foot_transform[i] = go1_config_matrix[i].inverse()*foot_position;
  }
  return foot_transform;
}

Eigen::Matrix<double, 4, 3> Quadruped::jointAnglesFromFootPositions(Eigen::Matrix<double, 4, 1> *foot_positions) {
  Eigen::Matrix<double, 4, 3> angles;
  for(uint8_t i = 0; i<4; i++) {
    double x = foot_positions[i][0];
    double y = foot_positions[i][1];
    double z = foot_positions[i][2];
    double F = sqrt(pow(x, 2) + pow(y, 2) - pow(l2, 2));
    double G = F - l1;
    double H = sqrt(pow(G, 2) + pow(z, 2));
    double theta1 = -atan2(y, x) - atan2(F, l2*pow(-1, i));
    double D = (pow(H, 2) - pow(l3, 2) - pow(l4, 2))/(2*l3*l4);
    double theta4 = -atan2(sqrt(1-pow(D, 2)),D);
    double theta3 = atan2(z,G) - atan2(l4*sin(theta4), l3 + l4*cos(theta4));
    angles.row(i) <<  theta1, theta3, theta4;     
  } 
  return angles;
}

sensor_msgs::JointState Quadruped::getJointStates() {
  sensor_msgs::JointState joint_states;
  joint_states.velocity.resize(12);
  joint_states.effort.resize(12);
  joint_states.header.stamp = ros::Time::now();
  joint_states.header.frame_id = "trunk";
  joint_states.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                       "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                       "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                       "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
                      };                     
  Eigen::Matrix<double, 4, 1>* foot_positions = footTransformsFromPositions();
  Eigen::Matrix<double, 4, 3>  joint_angles   = jointAnglesFromFootPositions(foot_positions);
  if (!joint_angles.allFinite()) {
    return joint_states;
  }
  for(uint8_t i = 0; i<4; i++) {
    joint_states.position.push_back(joint_angles.coeff(i, 0));
    joint_states.position.push_back(joint_angles.coeff(i, 1));
    joint_states.position.push_back(joint_angles.coeff(i, 2));
  }
  return joint_states;
}

sensor_msgs::JointState Quadruped::extractJointAngles() {
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "joint_states";
    joint_state.header.stamp = ros::Time::now();  
    joint_state.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};       
    for (size_t i = 0; i < 12; i++) {
        joint_state.position.push_back(base_low_state.motorState[i].q);
        joint_state.velocity.push_back(base_low_state.motorState[i].dq);
    }  
    return joint_state;
}

Eigen::Matrix3d Quadruped::rotx(double alpha) {
  Eigen::Matrix3d rx = Eigen::Matrix3d::Identity();
  rx << 1, 0,           0,
        0, cos(alpha), -sin(alpha),
        0, sin(alpha),  cos(alpha);
  return rx;
}

Eigen::Matrix3d Quadruped::roty(double beta) {
  Eigen::Matrix3d ry = Eigen::Matrix3d::Identity();
  ry << cos(beta), 0, sin(beta),
        0,         1, 0,
       -sin(beta), 0, cos(beta);
  return ry;
}

Eigen::Matrix3d Quadruped::rotz(double gamma) {
  Eigen::Matrix3d rz = Eigen::Matrix3d::Identity();
  rz << cos(gamma), -sin(gamma), 0,
        sin(gamma),  cos(gamma), 0,
        0,           0,          1;
  return rz;
}

Eigen::Matrix3d Quadruped::rotxyz(double alpha, double beta, double gamma) {
  Eigen::Matrix3d rxyz = Eigen::Matrix3d::Identity();
  rxyz = rotx(alpha) * (roty(beta) * rotz(gamma));
  return rxyz;
}
