/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Implementation of a unit test on the pelvis controller of the Pepper humanoid robot. The test consists in
 *  moving Pepper's hip and knee.
 *  This program uses extensively the actionlib from ROS. Kindly refer to the ROS tutorial on actions for a thorough understanding of the concept of
 *  actions in ROS. Actions works exactly like ROS services to the difference that they help you track the progress of your request. In the following 
 *  code we planned a motion using actions. Basically, we have a predefine set of waypoints that the robot will follow with regarding its actuators. 
 *  Before reaching the goal position, we are able to follow the positions  the robot before reaching the goal position.
 *  @file pelvis_control.cpp
 *  @details The program  aims to move the Pepper humanoid pelvis. This is done by defining a set of way points that 
 *           the robot will follow to move it pelvis to a certain position in space.
 *            
 *  @author Pamely ZANTOU. This program has been adapted from ROS tutorial on moving the ARI robot pelvis. There is another useful tutorial on moving
 *                         the PR2 robot pelvis.
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


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction>pelvis_control_client;

typedef boost::shared_ptr<pelvis_control_client>pelvis_control_client_ptr; 


// Create a ROS action client to move the pelvis of the Pepper robot 
void createPelvisClient (pelvis_control_client_ptr& actionClient)
{
    ROS_INFO("Creating action client to pelvis controller .....");

    actionClient.reset( new pelvis_control_client("/pepper_dcm/Pelvis_controller/follow_joint_trajectory", true) ); 


    int iterations     = 0; 
    int max_iterations = 3;
    
    // Wait for pelvis controller action server to come up 

    while( !actionClient -> waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations)  
    {
        ROS_DEBUG("Waiting for the pelvis controller to come up"); 
        ++ iterations; 
    }

    if( iterations == max_iterations)
    {
        throw std::runtime_error("Error in createPelvisClient: left pelvis controller action server not available"); 
    }

}


// Generate a simple trajectory with two waypoints to move Pepper's left hand 

void waypoints_pelvis_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    /* 
        - HipPitch
        - HipRoll
        - KneePitch
    */

    goal.trajectory.joint_names.push_back("HipPitch");
    goal.trajectory.joint_names.push_back("HipRoll");
    goal.trajectory.joint_names.push_back("KneePitch");


    

    // Two waypoints in this goal trajectory 
    goal.trajectory.points.resize(1);

    // First trajectory point 
    // Positions 
    int index = 0; 

    goal.trajectory.points[index].positions.resize(3);
    goal.trajectory.points[index].positions[0] = -0.02;  // HipPitch
    goal.trajectory.points[index].positions[1] = -0.02;   // HipRoll
    goal.trajectory.points[index].positions[2] =  0.00;   // KneePitch
  

    // Velocities 
    goal.trajectory.points[index].velocities.resize(3);
    for (int j = 0; j < 3; j++)
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
        ros::init(argc, argv, "pelvis_control"); 

        ROS_INFO("Start runnning pelvis control application .....");

        ros::NodeHandle nh;

        if(!ros::Time::waitForValid(ros::WallDuration(10.0))) //Important when using simulated clock 
        {
            ROS_FATAL("Time-out waiting for valud time");
            return EXIT_FAILURE;
        }

        // Create an pelvis controller action client to move the pelvis

       pelvis_control_client_ptr pelvis_client;
        createPelvisClient(pelvis_client);


        // Generate the goal for the Pepper's pelvis
        control_msgs::FollowJointTrajectoryGoal pelvis_goal; 
        waypoints_pelvis_goal(pelvis_goal);

        // Send the command to start the given trajectory 1s from now 
       pelvis_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); 
       pelvis_client -> sendGoal(pelvis_goal);

        // Wait for trajectory execution 
        while( !(pelvis_client -> getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds 
        }

        return 0;
}


