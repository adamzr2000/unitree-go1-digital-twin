#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
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

    // Set the control loop rate based on the retrieved parameter
    ros::Rate loop_rate(control_loop_rate);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	geometry_msgs::Twist twist;

	long motiontime = 0;

	std::ifstream myfile;
	char command;

	while (ros::ok())
	{

		motiontime += 2;

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

		pub.publish(twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
