#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_gesture_control");

    ros::NodeHandle nh;

    // Define the control loop rate parameter
    int control_loop_rate;

    // Retrieve the control loop rate parameter from the parameter server
    if (!nh.getParam("/node_gesture_control/control_loop_rate", control_loop_rate)) {
        // If parameter retrieval fails, use a default rate of 50 Hz -> 20ms
        control_loop_rate = 50;
        ROS_WARN_STREAM("Failed to retrieve control loop rate parameter. Using default value: " << control_loop_rate);
    }

    // Print the control loop rate for debugging
    ROS_INFO_STREAM("Control loop rate: " << control_loop_rate);

    ros::Rate loop_rate(control_loop_rate);


    // Retrieve cmd_vel topic name and stamped parameter
    std::string cmd_vel_topic;
    bool stamped;
    nh.param<std::string>("/node_gesture_control/cmd_vel", cmd_vel_topic, "/go1_controller/cmd_vel");
    nh.param("/node_gesture_control/stamped", stamped, true);

    ros::Publisher pub;
    if (stamped) {
        pub = nh.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic, 1);
    } else {
        pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    }

    geometry_msgs::Twist twist;
    geometry_msgs::TwistStamped twist_stamped;

    std::ifstream myfile;
    char command;
    long motiontime = 0;

    
    while (ros::ok())
        {
        motiontime += 2;  // Motion time increment every loop, was previously missing unit clarification.

        // Open "commands.txt" file to read commands from detected gestures
        myfile.open("/home/go1/app/command.txt");

        twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

        // Read command from file
        myfile >> command;

        std::cout << command << std::endl;

        // Process the command read from file
        if (command == 110) // 'n'
        {
            twist.linear.x = 0.0;
			// printf("move forward!\n");
        }
        else if (command == 100) // 'b'
        {
            twist.linear.x = -0.5;
			// printf("move backward!\n");
        }
        else if (command == 117) // 'u'
        {
            twist.linear.x = 0.5;
			// printf("move forward!\n");
        }
        else if (command == 108) // 'l'
        {
            twist.angular.z = 1.0;
			// printf("turn left!\n");
        }
        else if (command == 114) // 'r'
        {
            twist.angular.z = -1.0;
			// printf("turn right!\n");
        }

        // Close the file after reading command
        myfile.close();


        // Publish the message
        if (stamped) {
            twist_stamped.twist = twist;
            twist_stamped.header.stamp = ros::Time::now();
            twist_stamped.header.frame_id = "base";  // Typically the moving part of your robot
            pub.publish(twist_stamped);
        } else {
            pub.publish(twist);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}