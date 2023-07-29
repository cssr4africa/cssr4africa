/***************************************************************************************************
 * Pepper diagnostic routines: Implementation of unit tests for Pepper humanoid robot's wheel movements. 
 * It tests both translation (forward and backward) and rotation (clockwise and counterclockwise) functionalities.
 * To achieve this, the code publishes velocity commands on the cmd_vel topic to move the robot accordingly.
 * 
 * @file baseTranslationAndRotation.cpp
 * @author CSSR4Africa Team
 * @version 1.0
 * @date July 28, 2023
 * Audit Trail:
 *
*********************************************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baseTranslationAndRotation");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate rate(50);

    // Arrays to store the linear and angular velocity to move the robot, and the number of steps for each movement.
    double linear_velocities[]  = {0.1,  -0.1,     0,      0};
    double angular_velocities[] = {  0,     0,   0.7,   -0.7};
    int num_steps[]             = {150,   150,   100,    100};

    // Create a Twist message object to hold the velocity commands.
    geometry_msgs::Twist msg;

    //Loop through each movement.
    for (int i = 0; i < 4; ++i) {
        while (ros::ok() && num_steps[i] > 0) {
            // Set the linear and angular velocities for the current movement.
            msg.linear.x = linear_velocities[i];
            msg.angular.z = angular_velocities[i];
            
            // Publish the velocity command
            pub.publish(msg);
            
            // Output the velocity command on the terminal.
            ROS_INFO_STREAM("Velocity command: linear = " << msg.linear.x << " angular = " << msg.angular.z);
            
            rate.sleep();
            
            // Decrease the number of steps.
            num_steps[i]--;
        }
        
        // Pause after each movement before proceeding to the next one
        ros::Duration(2.0).sleep();
    }
    return 0;
}
