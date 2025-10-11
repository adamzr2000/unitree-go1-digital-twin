#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def mover():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz

    current = 0
    max = 0.86
    increment = 0.01
    sign = 1

    while not rospy.is_shutdown():
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']

        FR_hip_position = current + increment * sign
        current = FR_hip_position

        if (current > max):
            sign = -1
        elif (current < max * -1):
            sign = 1

        js.position = [0, 0.9, -1.853, 0, 0.9, -1.853, 0, 0.9, -1.853, 0, 0.9, -1.853]
        js.velocity = []
        js.effort = []
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass