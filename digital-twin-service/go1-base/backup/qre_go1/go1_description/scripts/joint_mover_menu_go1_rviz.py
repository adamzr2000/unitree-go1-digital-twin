#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


def set_stand_position(current_positions):
    # Set the fixed stand position
    current_positions['FR_hip_joint'] = 0
    current_positions['FR_thigh_joint'] = 0.9
    current_positions['FR_calf_joint'] = -1.853
    current_positions['FL_hip_joint'] = 0
    current_positions['FL_thigh_joint'] = 0.9
    current_positions['FL_calf_joint'] = -1.853
    current_positions['RR_hip_joint'] = 0
    current_positions['RR_thigh_joint'] = 0.9
    current_positions['RR_calf_joint'] = -1.853
    current_positions['RL_hip_joint'] = 0
    current_positions['RL_thigh_joint'] = 0.9
    current_positions['RL_calf_joint'] = -1.853


def mover():
    # Initialize ROS node and topic
    topic = 'joint_states'

    # Ask the user whether they want to use the /go1_gazebo/joint_states topic
    use_gazebo_topic = input("Do you want to use the gazebo joint_states topìc ? (yes/no): ").lower()
    if use_gazebo_topic == "yes":
        topic = 'go1_gazebo/joint_states'

    pub = rospy.Publisher(topic, JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10)  # 10hz

    # Define the initial positions for all joints
    current_positions = {
        'FR_hip_joint': 0,
        'FR_thigh_joint': 0,
        'FR_calf_joint': -1.853,
        'FL_hip_joint': 0,
        'FL_thigh_joint': 0,
        'FL_calf_joint': -1.853,
        'RR_hip_joint': 0,
        'RR_thigh_joint': 0,
        'RR_calf_joint': -1.853,
        'RL_hip_joint': 0,
        'RL_thigh_joint': 0,
        'RL_calf_joint': -1.853
    }

    # Define the maximum ranges for all joints
    # Conversion: (max*180)/3.14 = result-in-degrees -> range of (-result-in-degrees, +result-in-degrees)
    max_ranges = {
        'FR_hip_joint': 0.86,
        'FR_thigh_joint': 0.86,
        'FR_calf_joint': 0.86,
        'FL_hip_joint': 0.86,
        'FL_thigh_joint': 0.86,
        'FL_calf_joint': 0.86,
        'RR_hip_joint': 0.86,
        'RR_thigh_joint': 0.86,
        'RR_calf_joint': 0.86,
        'RL_hip_joint': 0.86,
        'RL_thigh_joint': 0.86,
        'RL_calf_joint': 0.86
    }

    increment = 0.01
    sign = 1


    # Ask the user whether they want to start in fixed stand position
    use_stand_position = input("Do you want to start in the fixed stand position? (yes/no): ").lower()
    if use_stand_position == "yes":
        set_stand_position(current_positions)


    # Display available joints for the user
    print("Available joints to move:")
    for joint_name in current_positions:
        print(f"- {joint_name}")
    joint_to_move = input("Enter the joint to move (or 'exit' to quit): ")

    if joint_to_move == 'exit':
        print("Exiting the program.")
        exit()

    while not rospy.is_shutdown():

        # Create a JointState message to publish joint positions
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = list(current_positions.keys())
 
        if joint_to_move in current_positions:

            joint_to_move_position = current_positions[joint_to_move] + increment * sign
            current_positions[joint_to_move] = joint_to_move_position
            
            # Loop moving the joints in the range (-max, +max)
            if (current_positions[joint_to_move] > max_ranges[joint_to_move]):
                sign = -1
            elif (current_positions[joint_to_move] < max_ranges[joint_to_move] * -1):
                sign = 1


            # Set positions for all joints
            js.position = [current_positions[joint] for joint in js.name]

            js.velocity = []
            js.effort = []
            pub.publish(js)
            rate.sleep()
        else:
            print("Invalid joint name. Please enter a valid joint name.")

if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
