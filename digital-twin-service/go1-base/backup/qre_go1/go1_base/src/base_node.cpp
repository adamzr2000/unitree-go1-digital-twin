#include <iostream>
#include "go1_base/base.hpp"

using namespace qre;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh, p_nh("~");
    Base* base_driver;
    UT::LoopFunc* control_loop;
    UT::InitEnvironment();

    int state_loop_rate;
    p_nh.param<int>("state_loop_rate", state_loop_rate, 50);  // Retrieve state_loop_rate parameter, default is 50Hz ~ 20ms

    float udp_send_dt;
    float udp_recv_dt;
    p_nh.param<float>("udp_send_dt", udp_send_dt, 0.002f);  // Retrieve state_loop_rate parameter, default is 2ms
    p_nh.param<float>("udp_recv_dt", udp_recv_dt, 0.002f);  // Retrieve state_loop_rate parameter, default is 2ms

    // Debug output
    ROS_INFO("state_loop_rate parameter value: %d", state_loop_rate);
    ROS_INFO("udp_send_dt parameter value: %.3f", udp_send_dt);
    ROS_INFO("udp_recv_dt parameter value: %.3f", udp_recv_dt);

    if (argc >= 2) {
        if (strcmp(argv[1], "high_level") == 0) {
            std::cout << "Working mode is set to: " << argv[1] << std::endl;
            UT::HighCmd cmd;
            UT::HighState state;
            base_driver  = new Base(cmd, state, &nh, &p_nh);
            control_loop = new UT::LoopFunc("control_loop", udp_send_dt, boost::bind(&Base::HighLevelControl, base_driver));
            // control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::HighLevelControl, base_driver));
        }
        else if (strcmp(argv[1], "low_level") == 0) {
            std::cout << "Working mode is set to: " << argv[1] << std::endl;                
            UT::LowCmd cmd;
            UT::LowState state;
            base_driver  = new Base(cmd, state, &nh, &p_nh);
            control_loop = new UT::LoopFunc("control_loop", udp_send_dt, boost::bind(&Base::LowLevelControl, base_driver));
            // control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::LowLevelControl, base_driver));
        }
        else {
            std::cout << "Invalid working mode" << std::endl;
            std::cout << "Supported modes are: high_level and low_level" << std::endl;
            exit(-1);
        }
    }
    else {
        std::cout << "Working mode not specified" << std::endl;
        std::cout << "Supported modes are: high_level and low_level" << std::endl;
        exit(-1);
    }    
    // UT::LoopFunc comm_bus_publisher("udp_send",  base_driver->dt, 3, boost::bind(&Base::packetSend,    base_driver));
    // UT::LoopFunc comm_bus_subscriber("udp_recv", base_driver->dt, 3, boost::bind(&Base::packetReceive, base_driver));
    UT::LoopFunc comm_bus_publisher("udp_send",  udp_send_dt, 3, boost::bind(&Base::packetSend,    base_driver));
    UT::LoopFunc comm_bus_subscriber("udp_recv", udp_recv_dt, 3, boost::bind(&Base::packetReceive, base_driver));
    comm_bus_publisher.start();
    comm_bus_subscriber.start();
    control_loop->start();
    ros::Rate loop_rate(state_loop_rate);  // Use the retrieved state_loop_rate parameter
    while (ros::ok()) {
        base_driver->publishStateMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
