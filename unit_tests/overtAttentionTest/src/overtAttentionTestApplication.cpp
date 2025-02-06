/* overtAttentionTestApplication.cpp
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
* This module is responsible for running the tests on the overt attention module.
* The tests are run using Google Test and the results are written to a file. 
* The module tests the scanning, social, seeking, location and disabled mode of the overtAttention node
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
* The attention mode to set the attention system to
* The location in the world to pay attention to in x, y, z coordinates
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform                      robot
* scanning                      true
* social                        true
* seeking                       true
* location                      true
* disabled                      true
* verboseMode                   true

...
* Subscribed Topics and Message Types
*
* /faceDetection/direction              faceDetection.msg     
* /robotLocalization/pose               sensor_msgs::JointState
* /soundDetection/data                  std_msgs::Float64    
* /naoqi_driver/camera/front/image_raw  sensor_msgs::ImageConstPtr     
* /overtAttention/mode                  Status.msg  
...
* Published Topics and Message Types
* 
* None

...
* Advertised Services
* 
* None

...
* Services Invoked
* 
* /overtAttention/set_mode                                         
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* overtAttentionTestOutput.dat
...
* Configuration Files
*
* overtAttentionTestConfiguration.ini
...
* Example Instantiation of the Module
*
* roslaunch unit_tests overtAttentionLaunchTestHarness
...
*
...
*
* Author: Muhammed Danso and Adedayo Akinade, Carnegie Mellon University Africa
* Email: mdanso@andrew.cmu.edu, aakinade@andrew.cmu.edu
* Date: January 10, 2025
* Version: v1.0
*
*/


#include "overtAttentionTest/overtAttentionTestInterface.h"

int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "overtAttentionTest", ros::init_options::NoSigintHandler);
    ::testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                   // The signal handler for the interrupt signal  

    node_name = ros::this_node::getName(); // Get the name of the node

    std::string copyright_message = node_name + "  " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t     This project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\t\t\t     Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t     Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t     This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                                      // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());                                                    // Print startup message

    // Check if the /speech topic is available.
    std::string speech_topic = "/speech";
    ROS_INFO("%s: creating a publisher for the speech commands...", node_name.c_str());
    while(ros::ok() && !is_topic_available(speech_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), speech_topic.c_str());
        speech_pub = nh.advertise<std_msgs::String>(speech_topic, 1, true);
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: created publisher for %s.", node_name.c_str(), speech_topic.c_str());

    // Create a subscriber object for faceDetection data
    std::string face_detection_topic = "/faceDetection/data";
    ROS_INFO("%s: subscribing to %s topic...\n", node_name.c_str(), face_detection_topic.c_str());
    while(ros::ok() && !is_topic_available(face_detection_topic)){
        ROS_ERROR_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...\n", node_name.c_str(), face_detection_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber face_detection_subscriber = nh.subscribe(face_detection_topic, 1, &face_detection_data_received);
    ROS_INFO("%s: subscribed to %s topic.\n", node_name.c_str(), face_detection_topic.c_str());

    // Create a subscriber object for overtAttention mode
    std::string engagement_mode_topic = "/overtAttention/mode";
    ROS_INFO("%s: subscribing to %s topic...\n", node_name.c_str(), engagement_mode_topic.c_str());
    while(ros::ok() && !is_topic_available(engagement_mode_topic)){
        ROS_ERROR_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...\n", node_name.c_str(), engagement_mode_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber mode_subscriber = nh.subscribe(engagement_mode_topic, 1, &mode_data_received);
    ROS_INFO("%s: subscribed to %s topic.\n", node_name.c_str(), engagement_mode_topic.c_str());


    string implementation_platform                  = "robot";
    bool verbose_mode                               = false;

    // Read the overt attention test input file
    if(read_overt_attention_test_configuration(&implementation_platform, &verbose_mode) != 0){
        ROS_ERROR("%s: error reading the overt attention test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }

    // Construct the full path of the configuration file
    #ifdef ROS
        test_report_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        test_report_path = "..";
    #endif

    // set configuration path
    test_report_path += "/overtAttentionTest/data/";
    test_report_path_and_file = test_report_path;
    test_report_path_and_file += test_report_file;

    // Clear the test report file and write the header before running any tests
    std::ofstream test_report(test_report_path_and_file, std::ios::out | std::ios::trunc); // Clear the file
    if (test_report.is_open()) {
        std::time_t now = std::time(nullptr);
        char date_time_string[100];
        // Write the header and the date and time of the test report
        if (std::strftime(date_time_string, sizeof(date_time_string), "%Y-%m-%d %H:%M:%S", std::localtime(&now))) {
            test_report << "Overt Attention Test Report: " << implementation_platform << "\n";
            test_report << "======================================\n";
            test_report << "Date: " << date_time_string << "\n\n";
        }
        test_report.close();
    } else {
        ROS_ERROR("%s: Failed to create test report file '%s'", node_name.c_str(), test_report_path_and_file.c_str());
    }

    if(implementation_platform == "robot"){
        ROS_INFO("%s: running on the robot platform...", node_name.c_str());
    }
    else{
        ROS_INFO("%s: running on the simulator platform...", node_name.c_str());
    }
    
    // Start the ROS spinner and run the tests
    ros::AsyncSpinner spinner(1); // Use async spinner to handle ROS callbacks
    spinner.start();

    // Run all the tests
    int result = 0;
    result = RUN_ALL_TESTS();

    // Close the test report file
    test_report.close();

    // Shutdown the ROS spinner and return the test result
    shut_down_handler(0);
    return result;
}