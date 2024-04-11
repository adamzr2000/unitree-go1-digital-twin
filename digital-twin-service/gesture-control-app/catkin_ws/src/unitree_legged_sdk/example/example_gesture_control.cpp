/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string.h>

// Use the namespace for UNITREE's SDK
using namespace UNITREE_LEGGED_SDK;

// Declare an ifstream object for file input
std::ifstream myfile;


// Custom class definition
class Custom
{
public:
    Custom(uint8_t level): 
      safe(LeggedType::Go1), 
      udp(level, 8090, "192.168.12.1", 8082){ // Initialize UDP communication with Go1's Rpi for high level commands
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0}; // Contains commands to control the robo9t
    HighState state = {0}; // Provides feedback on the current status of the robot
    uint8_t walk_flag=0;
    int motiontime = 0; // Timing Control for Actions 
    float dt = 0.002;  // Time delta for control loop (interval at which the control loop is executed) -> 2ms
    char key;
    char command;
    std::ifstream myfile;
    
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    // Increment motion time
    motiontime += 1;

    // Get the received Robot state via UDP
    udp.GetRecv(state); 

    // Print motion time and a state value
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
 
    // Open "commands.txt" file to read commands from detected gestures
    myfile.open("/home/go1/app/command.txt");

    // Initialize command structure with default values:

    // Set mode to idle
    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    //mode 3: position walk goto that position [0]=x [1]=y reference from dog frame at first time run
    
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;  // velocity[0]=forward [1]=sideward
    cmd.velocity[1] = 0.0f; 
    cmd.yawSpeed = 0.0f; // yawspeed >0=turn  <0=turn ==0=straight 
    cmd.reserve = 0;
    
    // Read command from file
    myfile >> command;
    
    std::cout << command << std::endl;

    // Process the command read from file
    if (command == 110) // 'n'
    {
        std::cout << "stand" << std::endl;
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
        
    }
    else if (command == 100) // 'b'
    {
        std::cout << "sit" << std::endl;
        cmd.mode = 1;
        cmd.bodyHeight = -0.5f;
    }
    else if (command == 117) // 'u'
    {
        std::cout << "go forward" << std::endl;
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        cmd.yawSpeed = 0;
        cmd.footRaiseHeight = 0.1;
    }
    else if (command == 108) // 'l'
    {
        std::cout << "turn left" << std::endl;
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        cmd.yawSpeed = 1;
        cmd.footRaiseHeight = 0.1;
    }
    else if (command == 114) // 'r'
    {
        std::cout << "turn right" << std::endl;
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        cmd.yawSpeed = -1;
        cmd.footRaiseHeight = 0.1;
    }

    // Close the file after reading command
    myfile.close();

    

    // if(motiontime > 0 && motiontime < 1000){
    //     cmd.mode = 1;
    //     cmd.euler[0] = -0.3;
    // }
    // if(motiontime > 1000 && motiontime < 2000){
    //     cmd.mode = 1;
    //     cmd.euler[0] = 0.3;
    // }
    // if(motiontime > 2000 && motiontime < 3000){
    //     cmd.mode = 1;
    //     cmd.euler[1] = -0.2;
    // }
    // if(motiontime > 3000 && motiontime < 4000){
    //     cmd.mode = 1;
    //     cmd.euler[1] = 0.2;
    // }
    // if(motiontime > 4000 && motiontime < 5000){
    //     cmd.mode = 1;
    //     cmd.euler[2] = -0.2;
    // }
    // if(motiontime > 5000 && motiontime < 6000){
    //     cmd.mode = 1;
    //     cmd.euler[2] = 0.2;
    // }
    // if(motiontime > 6000 && motiontime < 7000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = -0.2;
    // }
    // if(motiontime > 7000 && motiontime < 8000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.1;
    // }
    // if(motiontime > 8000 && motiontime < 9000){
    //     cmd.mode = 1;
    //     cmd.bodyHeight = 0.0;
    // }
    // if(motiontime > 9000 && motiontime < 11000){
    //     cmd.mode = 5;
    // }
    // if(motiontime > 11000 && motiontime < 13000){
    //     cmd.mode = 6;
    // }
    // if(motiontime > 13000 && motiontime < 14000){
    //     cmd.mode = 0;
    // }
    // if(motiontime > 14000 && motiontime < 18000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 2;
    //     cmd.velocity[0] = 0.4f; // -1  ~ +1
    //     cmd.yawSpeed = 2;
    //     cmd.footRaiseHeight = 0.1;
    //     // printf("walk\n");
    // }
    // if(motiontime > 18000 && motiontime < 20000){
    //     cmd.mode = 0;
    //     cmd.velocity[0] = 0;
    // }
    // if(motiontime > 20000 && motiontime < 24000){
    //     cmd.mode = 2;
    //     cmd.gaitType = 1;
    //     cmd.velocity[0] = 0.2f; // -1  ~ +1
    //     cmd.bodyHeight = 0.1;
    //     // printf("walk\n");
    // }
    // if(motiontime>24000 ){
    //     cmd.mode = 1;
    // } 

    // Send the command used to control the robot out via UDP
    udp.SetSend(cmd);
}

int main(void) 
{
    // Print initial messages
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Create a Custom object with high-level communication
    Custom custom(HIGHLEVEL);
    // InitEnvironment();

    // Create loop functions: 3 threads are created and 3 functions of the Custom class are bound to each of them 
    // UDPSend and UDPRecv are used for communication
    // We just put the control logic in RobotControl()
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    // Start the loops
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    // Main loop with a sleep delay
    while(1){
        sleep(10); 
    };

    return 0; 
}
