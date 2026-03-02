/* gestureExecutioTestApplication.cpp
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*
* <detailed functional description>
* This module is responsible for running the tests on the gesture execution module.
* The tests are run using Google Test and the results are written to a file. 
* The module tests the iconic, deictic, bow, nod, and symbolic gestures.
*
*
*
...
* Libraries
* Standard libraries
- std::string, std::vector, std::fstream
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h
*
*
*
...
* Parameters
*
* Command-line Parameters
*
* None
*
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform              |     robot
* iconic                      true
* deictic                     true
* bow                         true
* nod                         true
* symbolic                    false
*
*
*
...
* Subscribed Topics and Message Types
*
* None
...
* Published Topics and Message Types
* 
* /pepper/cmd_vel                                               geometry_msgs/Twist
*
*
*
...
* Input Data Files
*
* None
*
*
*
...
* Output Data Files
*
* gestureExecutionTestOutput.dat
*
*
*
...
* Configuration Files
*
* gestureExecutionTestConfiguration.ini
*
*
*
...
* Example Instantiation of the Module
*
* rosrun unit_tests gestureExecutionTest
*
*
...
*
* The launch file for the gesture execution unit tests is gestureExecutionTestLaunchTestHarness.launch.
*
* roslaunch unit_tests gestureExecutionTestLaunchTestHarness.launch
*
*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: January 10, 2025
* Version: v1.0
*
*/

#include "gestureExecutionTest/gestureExecutionTestInterface.h"

int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "gestureExecutionUnitTest", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ::testing::InitGoogleTest(&argc, argv);

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                   // The signal handler for the interrupt signal  

    node_name = ros::this_node::getName(); // Get the name of the node

    std::string copyright_message = node_name + "  " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t   This project is funded by the African Engineering and Technology Network (Afretec)\n\t\t\t\t\t\t   Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t   Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t   This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                                      // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());                                                    // Print startup message

    // Set the contents of the gesture execution configuration file
    string implementation_platform                  = "robot";
    string interpolation_type                       = "biological";
    string gesture_descriptors_config_filename      = "gestureDescriptorsTest.dat";
    string simulator_topics_filename                = "simulatorTopics.dat"; 
    string robot_topics_filename                    = "pepperTopics.dat";
    string verbose_mode_input                       = "true";
    bool verbose_mode = false;

    // Read the gesture execution test configuration file
    if(read_gesture_execution_test_configuration(&implementation_platform, &verbose_mode) != 0){
        ROS_ERROR("%s: error reading the gesture execution test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    
    // Check if the /gestureExecution/perform_gesture service is available
    std::string perform_gesture_service_name = "/gestureExecution/perform_gesture";
    while(ros::ok() && !is_service_available(perform_gesture_service_name)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s service to be available...", node_name.c_str(), perform_gesture_service_name.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s service available.", node_name.c_str(), perform_gesture_service_name.c_str());

    // Check if the /speech topic is available.
    std::string speech_topic = "/speech";
    ROS_INFO("%s: creating a publisher for the speech commands...", node_name.c_str());
    while(ros::ok() && !is_topic_available(speech_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), speech_topic.c_str());
        speech_pub = nh.advertise<std_msgs::String>(speech_topic, 1, true);
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: created publisher for %s.", node_name.c_str(), speech_topic.c_str());

    // Construct the full path of the data file
    #ifdef ROS
        test_report_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        test_report_path = "..";
    #endif

    // set configuration path
    test_report_path += "/gestureExecutionTest/data/";
    test_report_path_and_file = test_report_path;
    test_report_path_and_file += test_report_file;

    ROS_INFO("%s: Creating the test report file '%s'...", node_name.c_str(), test_report_path_and_file.c_str());

    // Clear the test report file and write the header before running any tests
    std::ofstream test_report(test_report_path_and_file, std::ios::out | std::ios::trunc); // Clear the file
    if (test_report.is_open()) {
        std::time_t now = std::time(nullptr);
        char date_time_string[100];
        // Write the header and the date and time of the test report
        if (std::strftime(date_time_string, sizeof(date_time_string), "%Y-%m-%d %H:%M:%S", std::localtime(&now))) {
            test_report << "Gesture Execution Test Report: " << implementation_platform << "\n";
            test_report << "======================================\n";
            test_report << "Date: " << date_time_string << "\n\n";
        }
        ROS_INFO("%s: Test report file '%s' created.", node_name.c_str(), test_report_path_and_file.c_str());
        test_report.close();
    } else {
        ROS_ERROR("%s: Failed to create test report file '%s'", node_name.c_str(), test_report_path_and_file.c_str());
        shut_down_handler(0);
        return 1;
    }

    verbose_mode_input = verbose_mode ? "true" : "false";
    // Write gesture execution configuration file
    if(write_configuration_file(implementation_platform, interpolation_type, gesture_descriptors_config_filename, simulator_topics_filename, robot_topics_filename, verbose_mode_input) != 0){
        ROS_ERROR("%s: error writing the gesture execution test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 1;
    }

    // Print the node ready message after initialization complete
    ROS_INFO("%s: ready.", node_name.c_str());

    // Start the ROS spinner and run the tests
    ros::AsyncSpinner spinner(1); // Use async spinner to handle ROS callbacks
    spinner.start();

    // Run all the tests
    int result = 0;
    result = RUN_ALL_TESTS();

    // Close the test report file
    test_report.close();

    // Rewrite the configuration file to its default values
    gesture_descriptors_config_filename      = "gestureDescriptors.dat";
    verbose_mode_input                       = "false";
    if(write_configuration_file(implementation_platform, interpolation_type, gesture_descriptors_config_filename, simulator_topics_filename, robot_topics_filename, verbose_mode_input) != 0){
        ROS_ERROR("%s: error writing the gesture execution test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 1;
    }

    // Shutdown the ROS spinner and return the test result
    shut_down_handler(0);
    return result;
}