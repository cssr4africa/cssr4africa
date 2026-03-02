#include "robotLocalization/robotLocalizationInterface.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotLocalization");
    ros::NodeHandle nh;
    
    // Create RobotLocalization instance
    RobotLocalization rl;
    ROS_INFO("Robot Localization Ready");
    
    double robot_initial_x = 4.4;
    double robot_initial_y = 7.8;
    double robot_initial_theta = 270.0;
    
    if (argc > 1)
    {
        robot_initial_x = std::stod(argv[1]);
        robot_initial_y = std::stod(argv[2]);
        robot_initial_theta = std::stod(argv[3]);
    }
    else
    {
        ROS_INFO("Initial pose provided (x: %.2f, y: %.2f, theta: %.2f)", 
                 robot_initial_x, robot_initial_y, robot_initial_theta);
    }
    
    // Print log message similar to what we would want if the service was called
    ROS_INFO("Initializing robot pose to (x: %.2f, y: %.2f, theta: %.2f)", 
             robot_initial_x, robot_initial_y, robot_initial_theta);
    
    // Initialize the RobotLocalization class
    rl.setInitialValues(robot_initial_x, robot_initial_y, robot_initial_theta);
    
    // Main loop
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(10, "robotLocalization: running");
        ros::spinOnce();
    }
    
    // Clean up parameters
    if (nh.hasParam("initial_robot_x"))
    {
        nh.deleteParam("initial_robot_x");
    }
    if (nh.hasParam("initial_robot_y"))
    {
        nh.deleteParam("initial_robot_y");
    }
    if (nh.hasParam("initial_robot_theta"))
    {
        nh.deleteParam("initial_robot_theta");
    }
    return 0;
}
