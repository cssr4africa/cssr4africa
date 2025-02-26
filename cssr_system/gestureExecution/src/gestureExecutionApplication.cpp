/* gestureExecutionApplication.cpp
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
* This module is responsible for hosting the service that executes the gestures on the robot. 
* The module receives the gesture type, gesture ID, gesture duration, bow/nod angle, 
* and the location in the world to pay attention/point to in x, y, z coordinates.
* The module then executes the gesture based on the received parameters. 
*
* The module supports the execution of deictic, iconic, symbolic, bow, and nod gestures.
* The iconic gestures currently supported are welcome and wave (goodbye) gestures. 
* 
* The module also supports the selection of the implementation platform (simulator or robot) 
* and the interpolation type (linear or biological motion).
* 
* The module is implemented as a ROS service that receives the gesture parameters 
* and returns the status of the gesture execution.
*
* The gestures could either be executed using linear velocity interpolation or a model of biological motion (minimum-jerk model).
*
* The module subscribes to the /sensor_msgs/joint_states topic to receive the joint states of the robot.
* The module also subscribes to the /robotLocalization/pose topic to receive the coordinates of the robot in the world.
* 
* The module is implemented in C++ and uses the ROS libraries for communication with the robot.
* The module is part of the CSSR4A package and is used to execute gestures on the robot.
*
*
*
...
* Libraries
* Standard libraries
- std::string, std::vector, std::fstream, std::pow, std::sqrt, std::abs
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
* platform                    robot
* interpolation               biological
* gestureDescriptors          gestureDescriptors.dat
* simulatorTopics             simulatorTopics.dat
* robotTopics                 pepperTopics.dat
* verboseMode                 true
*
*
*
...
* Subscribed Topics and Message Types
*
* /sensor_msgs/joint_states
* /robotLocalization/pose
...
* Published Topics and Message Types
*
* None
...
* Advertised Service 
*
* /gestureExecution/perform_gesture
* 
...
* Invoked Service
* 
* /overtAttention/set_mode
*
*
*
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
* gestureDescriptors.dat
*
*
*
...
* Output Data Files
*
* None
*
*
*
...
* Configuration Files
*
* gestureExecutionConfiguration.ini
*
*
*
...
* Example Instantiation of the Module
*
* rosrun gestureExecution perform_gesture
*
*
...
*
* The clients can invoke the service by providing the gesture type, gesture ID, gesture duration, bow_nod angle, 
* and the location in the world to pay attention/point to in x, y, z coordinates.
* The service will execute the gesture based on the received parameters and return the status of the gesture execution.
*
* Examples of calling the service is shown below:
* ----- rosservice call /perform_gesture -- deictic 01 3000 25 3.6 2.5 0.82
* This will execute a pointing gesture with a duration of 3000 ms, and the location in the world to point to in x, y, z coordinates.
*
* ----- rosservice call /perform_gesture -- bow 01 3000 25 3.6 2.5 0.82
* This will execute a pointing gesture with a duration of 3000 ms, and bow at an angle of 45 degrees.
*
*
*
*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: January 10, 2025
* Version: v1.0
*
*/

#include "gestureExecution/gestureExecutionInterface.h"

int main(int argc, char **argv){
    // Initialize the ROS node
    ros::init(argc, argv, "gestureExecution", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    // Get the name of the node
    node_name = ros::this_node::getName();

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                                      // The signal handler for the interrupt signal

    std::string copyright_message = node_name + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t   This project is funded by the African Engineering and Technology Network (Afretec)\n\t\t\t\t\t\t   Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t   Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t   This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                                      // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());                                                    // Print startup message

    // Initialize the node: Read configuration file and set the implementation platform, interpolation type, and topics file name
    int node_initialization_status = 0;
    node_initialization_status = initialize_node(&implementation_platform, &interpolation_type, &interpolation_mode, &gesture_descriptors_config_filename, &simulator_topics_filename, &robot_topics_filename, &topics_file_name, &verbose_mode_input, &verbose_mode);
    if(node_initialization_status != 0){
        ROS_ERROR("%s: initialization failed due to incorrect configuration.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }

    // Extract the joint_states topic and subscribe to the joint states topic
    std::string joint_states_topic;
    if(extract_topic("JointStates", topics_file_name, &joint_states_topic)){
        ROS_ERROR("%s: error extracting the joint states topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s...", node_name.c_str(), joint_states_topic.c_str());
    while(ros::ok() && !is_topic_available(joint_states_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), joint_states_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber joint_states_subscriber = n.subscribe(joint_states_topic, 1, &joint_states_message_received);
    ROS_INFO("%s: subscribed to %s.", node_name.c_str(), joint_states_topic.c_str());

    // Create a publisher for the velocity commands
    std::string velocity_topic;
    if(extract_topic("Wheels", topics_file_name, &velocity_topic)){
        ROS_ERROR("%s: error extracting the wheels topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: creating a publisher for the velocity commands...", node_name.c_str());
    while(ros::ok() && !is_topic_available(velocity_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), velocity_topic.c_str());
        gesture_velocity_publisher = n.advertise<geometry_msgs::Twist>(velocity_topic.c_str(), 1, true);
        ros::Duration(1).sleep();
    }
    // gesture_velocity_publisher = n.advertise<geometry_msgs::Twist>(velocity_topic.c_str(), 1000, true);
    ROS_INFO("%s: created a publisher for the velocity commands.", node_name.c_str());

    // Subscribe to the /robotLocalization/pose topic
    std::string robot_pose_topic;
    if(extract_topic("RobotPose", topics_file_name, &robot_pose_topic)){
        ROS_ERROR("%s: error extracting the robot pose topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to robot pose topic...", node_name.c_str());
    while(ros::ok() && !is_topic_available(robot_pose_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), robot_pose_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber robot_pose_subscriber = n.subscribe(robot_pose_topic, 1, &robot_pose_message_received);
    ROS_INFO("%s: subscribed to robot pose topic.", node_name.c_str());

    // Check if the overtAttention/set_mode service is available
    std::string overt_attention_service_name = "/overtAttention/set_mode";
    ROS_INFO("%s: creating client for the %s service...", node_name.c_str(), overt_attention_service_name.c_str());
    while(ros::ok() && !is_service_available(overt_attention_service_name)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s service to be available...", node_name.c_str(), overt_attention_service_name.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: created client for the %s service.", node_name.c_str(), overt_attention_service_name.c_str());
    overt_attention_client = n.serviceClient<cssr_system::setMode>(overt_attention_service_name); 

    // Advertise the /gestureExecution/perform_gesture service
    std::string perform_gesture_service_name = "/gestureExecution/perform_gesture";
    ros::ServiceServer perform_gesture_service;                                                     // perform gesture service object
    ROS_INFO("%s: advertising the %s service...", node_name.c_str(), perform_gesture_service_name.c_str());
    while(ros::ok() && !perform_gesture_service){
        perform_gesture_service = n.advertiseService(perform_gesture_service_name, execute_gesture);
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: advertised the %s service...", node_name.c_str(), perform_gesture_service_name.c_str());
    

    // Print the node ready message after initialization complete
    ROS_INFO("%s: initialization complete...", node_name.c_str());
    node_initialized = true;                                                                        // Set the node initialization flag to true


    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO_THROTTLE(OPERATION_INFO_PERIOD, "%s: running...", node_name.c_str());              // Print heartbeat message
    }

    return 0;
}