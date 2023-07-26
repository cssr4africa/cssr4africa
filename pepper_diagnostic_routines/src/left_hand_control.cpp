/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Implementation of a unit test on the left hand of the Pepper humanoid robot. The test consists in moving 
 *  Pepper's left hand.
 *  This program uses extensively the actionlib from ROS. Kindly refer to the ROS tutorial on actions for a thorough understanding of the concept
 *  actions in ROS. Actions works exactly like ROS service to the difference that they help you track the progress of your request. In the following 
 *  code we planned a motion using actions. Basically, we have a predefine set of waypoints that the robot will follow with regarding its actuators. 
 *  Before reaching the goal position,  we are able to follow the positions  the robot before reaching the goal position.
 *  @file left_hand_control.cpp
 *  @details The program  aims to move the Pepper humanoid head. This is done by defining a set of way points that 
 *           the robot will follow to move it head to a certain position in space.
 *            
 *  @author Pamely ZANTOU. This program has been adapted from ROS tutorial on moving the ARI robot head. There is another useful tutorial on moving
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


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction>left_hand_control_client;

typedef boost::shared_ptr<left_hand_control_client>left_hand_control_client_ptr; 


// Create a ROS action client to move the hand of the Pepper robot 
void createLeftHandClient (left_hand_control_client_ptr& actionClient)
{
    ROS_INFO("Creating action client to hand controller .....");

    actionClient.reset( new left_hand_control_client("/pepper_dcm/LeftHand_controller/follow_joint_trajectory", true) ); 


    int iterations     = 0; 
    int max_iterations = 1;
    
    // Wait for hand controller action server to come up 

    while( !actionClient -> waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations)  
    {
        ROS_DEBUG("Waiting for the left hand controller to come up"); 
        ++ iterations; 
    }

    if( iterations == max_iterations)
    {
        throw std::runtime_error("Error in createLeftHandClient: left hand controller action server not available"); 
    }

}


// Generate a simple trajectory with two waypoints to move Pepper's left hand 

void waypoints_left_hand_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    /* 
        - RHand
    */
    goal.trajectory.joint_names.push_back("LHand");

    

    // Two waypoints in this goal trajectory 
    goal.trajectory.points.resize(1);

    // First trajectory point 
    // Positions 
    int index = 0; 

    goal.trajectory.points[index].positions.resize(1);
    goal.trajectory.points[index].positions[0] = 0.66;  // LHand
  

    // Velocities 
    goal.trajectory.points[index].velocities.resize(1);
    goal.trajectory.points[index].velocities[0] = 0.67; 

   

    // To be reached 2 seconds after starting along the trajectory 
    goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

    
}


    // main program 

int main(int argc, char** argv)
{

        // Init ROS node 
        ros::init(argc, argv, "left_hand_control"); 

        ROS_INFO("Start runnning left hand control application .....");

        ros::NodeHandle nh;

        if(!ros::Time::waitForValid(ros::WallDuration(10.0))) //Important when using simulated clock 
        {
            ROS_FATAL("Time-out waiting for value time");
            return EXIT_FAILURE;
        }

        // Create the left hand controller action client to move the fingers

       left_hand_control_client_ptr left_hand_client;
        createLeftHandClient(left_hand_client);


        // Generate the goal for the Pepper's left hand
        control_msgs::FollowJointTrajectoryGoal left_hand_goal; 
        waypoints_left_hand_goal(left_hand_goal);

        // Send the command to start the given trajectory 1s from now 
       left_hand_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); 
       left_hand_client -> sendGoal(left_hand_goal);

        // Wait for trajectory execution 
        while( !(left_hand_client -> getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds 
        }

        return 0;
}


