#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import sys

class OdomAxisCheck:
    def __init__(self, axis):
        rospy.init_node('odom_axis_check', anonymous=True)
        self.axis = axis
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.initial_position = None
        self.final_position = 0.0
        rospy.on_shutdown(self.calculate_displacement)

    def odom_callback(self, data):
        current_position = getattr(data.pose.pose.position, self.axis)
        if self.initial_position is None:
            self.initial_position = current_position
            rospy.loginfo("Initial %s position captured: %s meters", self.axis, self.initial_position)
        self.final_position = current_position

    def calculate_displacement(self):
        if self.initial_position is not None:
            displacement = self.final_position - self.initial_position
            rospy.loginfo("Final %s position: %s meters", self.axis, self.final_position)
            rospy.loginfo("Displacement along %s: %s meters", self.axis, displacement)
        else:
            rospy.loginfo("Initial position not captured; cannot calculate displacement.")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: rosrun your_package_name odom_axis_check.py [x/y]")
        sys.exit(1)
    axis = sys.argv[1]
    if axis not in ['x', 'y']:
        print("Invalid axis. Please specify 'x' or 'y'.")
        sys.exit(1)
    try:
        odom_checker = OdomAxisCheck(axis)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
