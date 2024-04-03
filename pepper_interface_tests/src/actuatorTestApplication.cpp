/* actuatorTestApplication.cpp
*
* <detailed functional description>
* The component test the functionality of the actuator of the robot using the ROS interface.
* The test is performed by sending commands to the robot and checking if the robot performs the
* expected action. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously. 

...
* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h

...
* Parameters
*
* Command-line Parameters
*
* None
...
* Configuration File Parameters

* Key | Value 
* --- | ---
* platform        | robot
* simulatorTopics | simulatorTopics.dat
* robotTopics     | pepperTopics.dat
* mode            | sequential
*
* Key | Value
* --- | ---
* Head   | true
* RArm   | true
* LArm   | true
* RHand  | true
* LHand  | true
* Leg    | true
* Wheels | false

...
* Subscribed Topics and Message Types
*
* None
...
* Published Topics and Message Types
* 
* /pepper_dcm/Head_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper_dcm/RightArm_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftArm_controller/follow_joint_trajectory        trajectory_msgs/JointTrajectory
* /pepper_dcm/RightHand_controller/follow_joint_trajectory      trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftHand_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/Pelvis_controller/follow_joint_trajectory         trajectory_msgs/JointTrajectory
* /cmd_vel                                                      geometry_msgs/Twist

* /pepper/Head_controller/follow_joint_trajectory               trajectory_msgs/JointTrajectory
* /pepper/RightArm_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper/LeftArm_controller/follow_joint_trajectory            trajectory_msgs/JointTrajectory
* /pepper/Pelvis_controller/follow_joint_trajectory             trajectory_msgs/JointTrajectory
* /pepper/cmd_vel                                               geometry_msgs/Twist
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* None
...
* Configuration Files
*
* actuatorTestConfiguration.ini
* actuatorTestInput.ini
...
* Example Instantiation of the Module
*
* rosrun pepper_interface_tests actuatorTest
...
*
* Author: Yohannes Tadesse Haile, Carnegie Mellon University Africa
* Email: yohanneh@andrew.cmu.edu
* Date: March 19, 2024
* Version: v1.0
*
*/

# include "pepper_interface_tests/actuatorTest.h"

int main(int argc, char** argv) {
    std::vector<std::string> testName = extractTests("actuator");

    // Initialize ROS
    ros::init(argc, argv, "actuatorTest");
    ros::NodeHandle nh;

    // Extract the mode to run the tests
    std::string mode = extractMode();
    printf("Mode: %s\n", mode.c_str());

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }
    
    if (mode == "sequential"){
        executeTestsSequentially(testName, nh);
    } else if (mode == "parallel"){
        executeTestsInParallel(testName, nh);
    }   
    else{
        printf("Invalid mode. Please check the mode in the configuration file.\n");
        promptAndExit(1);
    }
          
    return 0;
}
