/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Implementation of a unit test on the right hand of the Pepper humanoid robot. The test consists in moving 
 *  Pepper's right hand.
 *  This program uses extensively the actionlib from ROS. Kindly refer to the ROS tutorial on actions for a thorough understanding of the concept
 *  actions in ROS. Actions works exactly like ROS service to the difference that they help you track the progress of your request. In the following 
 *  code we planned a motion using actions. Basically, we have a predefine set of waypoints that the robot will follow with regarding its actuators. 
 *  Before reaching the goal position,  we are able to follow the positions  the robot before reaching the goal position.
 *  @file right_hand_control.cpp
 *  @details The program  aims to move the Pepper humanoid head. This is done by defining a set of way points that 
 *           the robot will follow to move it head to a certain position in space.
 *            
 *  @author Pamely ZANTOU. This program has been adapted from ROS tutorial on moving the ARI robot head. There is another useful tutorial on moving
 *                         the PR2 robot head.
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


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction>right_hand_control_client;

typedef boost::shared_ptr<right_hand_control_client>right_hand_control_client_ptr; 


// Create a ROS action client to move the handd of the Pepper robot 
void createRightHandClient (right_hand_control_client_ptr& actionClient)
{
    ROS_INFO("Creating action client to right hand controller .....");

    actionClient.reset( new right_hand_control_client("/pepper_dcm/RightHand_controller/follow_joint_trajectory", true) ); 


    int iterations     = 0; 
    int max_iterations = 1;
    
    // Wait for hand controller action server to come up 

    while( !actionClient -> waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations)  
    {
        ROS_DEBUG("Waiting for the right hand controller to come up"); 
        ++ iterations; 
    }

    if( iterations == max_iterations)
    {
        throw std::runtime_error("Error in createRightHandClient: left hand controller action server not available"); 
    }

}


// Generate a simple trajectory with two waypoints to move Pepper's left hand 

void waypoints_right_hand_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    /* 
        - RHand
    */
    goal.trajectory.joint_names.push_back("RHand");

    

    // Two waypoints in this goal trajectory 
    goal.trajectory.points.resize(1);

    // First trajectory point 
    // Positions 
    int index = 0; 

    goal.trajectory.points[index].positions.resize(1);
    goal.trajectory.points[index].positions[0] = 0.66;  // RHand
  

    // Velocities 
    goal.trajectory.points[index].velocities.resize(1);
    goal.trajectory.points[index].velocities[0] = 0.05; 

   

    // To be reached 2 seconds after starting along the trajectory 
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    
}


    // main program 

int main(int argc, char** argv)
{

        // Init ROS node 
        ros::init(argc, argv, "right_hand_control"); 

        ROS_INFO("Start runnning right hand control application .....");

        ros::NodeHandle nh;

        if(!ros::Time::waitForValid(ros::WallDuration(10.0))) //Important when using simulated clock 
        {
            ROS_FATAL("Time-out waiting for valud time");
            return EXIT_FAILURE;
        }

        // Create an hand controller action client to move the fingers

       right_hand_control_client_ptr right_hand_client;
        createRightHandClient(right_hand_client);


        // Generate the goal for the Pepper's right hand
        control_msgs::FollowJointTrajectoryGoal right_hand_goal; 
        waypoints_right_hand_goal(right_hand_goal);

        // Send the command to start the given trajectory 1s from now 
       right_hand_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); 
       right_hand_client -> sendGoal(right_hand_goal);

        // Wait for trajectory execution 
        while( !(right_hand_client -> getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds 
        }

        return 0;
}


