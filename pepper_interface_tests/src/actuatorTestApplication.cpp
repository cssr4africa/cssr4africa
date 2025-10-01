/* actuatorTestApplication.cpp Application code for running the actuator tests on Pepper robot
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.

* The component test the functionality of the actuator of the robot using the ROS interface.
* The test is performed by sending commands to the robot and checking if the robot performs the
* expected action. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously. 

* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h

* Parameters
*
* Command-line Parameters
*
* None

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


* Subscribed Topics and Message Types
*
* None

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

* Services Invoked
*
* None

* Services Advertised and Request Message
* 
* None

* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
* actuatorTestInput.dat

* Output Data Files
*
* None

* Configuration Files
*
* actuatorTestConfiguration.ini

* Example Instantiation of the Module
*
* rosrun pepper_interface_tests actuatorTest

*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa, Carnegie Mellon University Africa
* Email: yohanneh@andrew.cmu.edu
* Date: September 25, 2025
* Version: v1.1
*
*/

# include "pepper_interface_tests/actuatorTestInterface.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "actuatorTest");
    ros::NodeHandle nh;

    // Start an async spinner so timers/callbacks keep firing even if tests block
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Get the name of the node (without leading '/')
    const std::string node_name = cleanNodeName(ros::this_node::getName());

    // Heartbeat every 10 seconds
    ros::Timer heartbeat = nh.createTimer(ros::Duration(10.0), heartbeatCb);

    std::string software_version = "v1.1";

    std::string copyright_message =
        " " + node_name + ": " + software_version +
        "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
        "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme."
        "\n\t\t\t\tWebsite: www.cssr4africa.org"
        "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());
    ROS_INFO(" %s: startup.", node_name.c_str());

    std::vector<std::string> testName = extractTests();

    // Extract the mode to run the tests
    std::string mode = extractMode();
    if (verboseMode) {
        printf("Mode: %s\n", mode.c_str());
    }

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timeout waiting for valid time");
        return EXIT_FAILURE; 
    }

    // Run tests
    if (mode == "sequential") {
        executeTestsSequentially(testName, nh);
    } else if (mode == "parallel") {
        executeTestsInParallel(testName, nh);
    } else {
        printf("Invalid mode. Please check the mode in the configuration file.\n");
        promptAndExit(1);
    }

    // Give timers/logs a chance to flush if tests return immediately
    ros::Duration(0.1).sleep();

    return 0;
}
