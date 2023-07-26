/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Implementation of a unit test on the right arm of the Pepper humanoid robot. The test consists in moving the arm.
 *  This program uses extensively the actionlib from ROS. Kindly refer to the ROS tutorial on actions for a thorough understanding of the concept
 *  actions in ROS. Actions works exactly like ROS service to the difference that they help you track the progress of your request. In the following 
 *  code we planned a motion using actions. Basically, we have a predefine set of waypoints that the robot will follow with regarding its actuators. 
 *  Before reaching the goal position,  we are able to follow the positions  the robot before reaching the goal position.
 *  @file right_arm_control.cpp
 *  @details The program  aims to move the Pepper humanoid right arm. This is done by defining a set of way points that 
 *           the robot will follow to move it arm to a certain position in space.
 *            
 *  @author Pamely ZANTOU. This program has been adapted from ROS tutorial on moving the ARI robot arm. There is another useful tutorial on moving
 *                         the PR2 robot arm.
 *    
 *  @version 1.0 
 *  @date March 2023
 * 
 *  Audit Trail
 *  -----------
 * 
 * 
 * ********************************************************************************************************************************************/


// C++ standard headers 

#include <exception>
#include <string>


// Boost headers 
#include <boost/shared_ptr.hpp>

// ROS headers 
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> right_arm_control_client;

typedef boost::shared_ptr<right_arm_control_client> right_arm_control_client_ptr; 


// Create a ROS action client to move the right arm of the Pepper robot 
void createRightArmClient (right_arm_control_client_ptr& actionClient)
{
    ROS_INFO("Creating action client to arm controller .....");

    actionClient.reset( new right_arm_control_client("/pepper_dcm/RightArm_controller/follow_joint_trajectory", true) ); 


    int iterations     = 0; 
    int max_iterations = 5;
    
    // Wait for arm controller action server to come up 

    while( !actionClient -> waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations)  
    {
        ROS_DEBUG("Waiting for the arm controller to come up"); 
        ++ iterations; 
    }

    if( iterations == max_iterations)
    {
        throw std::runtime_error("Error in createRightArmClient: right arm controller action server not available"); 
    }

}


// Generate a simple trajectory with two waypoints to move Pepper's right hand 

void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    /* 
        - RShoulderPitch
        - RShoulderRoll
        - RElbowYaw
        - RElbowRoll
        - RWristYaw
    
    */
    goal.trajectory.joint_names.push_back("RElbowRoll");
    goal.trajectory.joint_names.push_back("RElbowYaw");
    goal.trajectory.joint_names.push_back("RShoulderPitch");
    goal.trajectory.joint_names.push_back("RShoulderRoll");
    goal.trajectory.joint_names.push_back("RWristYaw");

    

    // Two waypoints in this goal trajectory 
    goal.trajectory.points.resize(1);

    // First trajectory point 
    // Positions 
    int index = 0; 

    goal.trajectory.points[index].positions.resize(5);
    goal.trajectory.points[index].positions[0] = 0.13;  // RElbowRoll
    goal.trajectory.points[index].positions[1] = 0.00;   // RElbowYaw
    goal.trajectory.points[index].positions[2] = 0.54;  // RShoulderPitch
    goal.trajectory.points[index].positions[3] = -0.10; // RShoulderRoll 
    goal.trajectory.points[index].positions[4] = 1.06;  // RWristYaw
  

    // Velocities 
    goal.trajectory.points[index].velocities.resize(5);
    for (int j = 0; j < 5; j++)
    {
        goal.trajectory.points[index].velocities[j] = 0.05; 
    }   

   

    // To be reached 2 seconds after starting along the trajectory 
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    
}


    // main program 

int main(int argc, char** argv)
{

        // Init ROS node 
        ros::init(argc, argv, "right_arm_control"); 

        ROS_INFO("Start runnning right arm control application .....");

        ros::NodeHandle nh;

        if(!ros::Time::waitForValid(ros::WallDuration(10.0))) //Important when using simulated clock 
        {
            ROS_FATAL("Time-out waiting for valud time");
            return EXIT_FAILURE;
        }

        // Create an arm controller action client to move the arm

        right_arm_control_client_ptr right_arm_client;
        createRightArmClient(right_arm_client);


        // Generate the goal for the Pepper's right arm 
        control_msgs::FollowJointTrajectoryGoal right_arm_goal; 
        waypoints_arm_goal(right_arm_goal);

        // Send the command to start the given trajectory 1s from now 
        right_arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); 
        right_arm_client -> sendGoal(right_arm_goal);

        // Wait for trajectory execution 
        while( !(right_arm_client -> getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds 
        }

        return 0;
}


