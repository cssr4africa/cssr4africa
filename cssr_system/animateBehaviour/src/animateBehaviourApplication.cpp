/*
 * animateBehaviourApplication.cpp
 * Animate behavior controller for creating lifelike robot movements
 * 
 * Implements a ROS node that generates subtle autonomous movements to make the robot
 * appear more lifelike and animate. The system manages three types of movements:
 * - Subtle body joint movements
 * - Hand flexing movements
 * - Small base rotations around the z-axis
 * 
 * Key features:
 * - Maintains movements near home positions using randomized patterns
 * - Configurable movement ranges (as percentage of joint ranges)
 * - No head control
 * - Supports selective enabling of different movement types
 * - Can be enabled/disabled via ROS service for social interaction coordination
 * - Configurable via external topic mapping files for physical/simulated robots
 * - Optional verbose mode for movement debugging
 * 
 * Libraries:
 *   - ROS core libraries:
 *     - roscpp
 *     - ros/package.h
 *     - actionlib
 *     - control_msgs
 *     - trajectory_msgs
 *     - geometry_msgs
 *   - Standard C++ libraries:
 *     - iostream
 *     - fstream
 *     - thread
 *     - chrono
 *     - vector
 *     - map
 *     - string
 *     - atomic
 *     - random
 * 
 * Parameters:
 *   ROS Parameters:
 *   - None
 * 
 * Configuration File Parameters (animateBehaviourConfiguration.ini):
 *   - platform: Target platform ("robot" or "simulator")
 *   - behaviour: Type of animation behavior ("body", "hands", "rotation", "All")
 *   - simulatorTopics: Topic mapping file for simulator ("simulatorTopics.dat")
 *   - robotTopics: Topic mapping file for robot ("pepperTopics.dat")
 *   - verboseMode: Enable/disable debug output (default: false)  
 *   - rotMaximumRange	 
 *   - selectedRange
 *   - armMaximumRange		
     - handMaximumRange				
     - legMaximumRange				
     - gestureDuration				
     - numPoints						
     - numPointsLeg				
     - legRepeatFactor			
 * 
 * Subscribed Topics and Message Types:
 *   - None
 * 
 * Published Topics and Message Types:
 * - None (This node does not publish any topics directly)
 * 
 * Topics Used By Node (But Not Published):
 *   - Action Client Topics (Node sends goals to these action servers):
 *      Physical Robot:
 *           - /pepper_dcm/RightHand_controller/follow_joint_trajectory
 *           - /pepper_dcm/LeftHand_controller/follow_joint_trajectory
 *           - /pepper_dcm/RightArm_controller/follow_joint_trajectory
 *           - /pepper_dcm/LeftArm_controller/follow_joint_trajectory
 *           - /pepper_dcm/Pelvis_controller/follow_joint_trajectory
 * 
 *    Simulator:
 *           - /pepper/RightArm_controller/follow_joint_trajectory
 *           - /pepper/LeftArm_controller/follow_joint_trajectory
 *           - /pepper/Pelvis_controller/follow_joint_trajectory
 * 
 * Topics Used via Publishers (Node sends messages but doesn't publish topics):
 *    Physical Robot:
 *          - /pepper_dcm/cmd_moveto
 * 
 *    Simulator:
 *          - /pepper/cmd_vel
 * 
 * Note: It sends action goals and movement commands through established topics but does
 * not create or publish any topics of its own.
 * 
 * Services Advertised:
 *   - animateBehaviour/setActivation (cssr_system/setActivation)
 *      Request: string state ("enabled" or "disabled")
 *      Response: bool success
 *
 * Services Invoked:
 *  - None
 * 
 * Input Data Files:
 *   - pepperTopics.dat:
 *   - simulatorTopics.dat:
 * 
 * Output Data Files:
 *  - animateBehaviourLogFile.log: Log file for runtime messages
 * 
 * Configuration Files
 *   - animateBehaviourConfiguration.ini:
 *
 * Debug Settings:
 *   - verboseMode: Enable/disable debug output (default: false)
 *   
 *   Range Parameters:
 *   - rotMaximumRange: Maximum rotation range for base movement 
 *   - selectedRange: Selected movement range as fraction of maximum 
 *   - armMaximumRange: Maximum range for each arm joint [5 values] 
 *   - handMaximumRange: Maximum range for hand movement 
 *   - legMaximumRange: Maximum range for each leg joint [3 values] 
 *   
 *   Movement Parameters:
 *   - gestureDuration: Duration of each movement in seconds 
 *   - numPoints: Number of points for arm and hand movements 
 *   - numPointsLeg: Number of points for leg movements 
 *   - legRepeatFactor: Number of repetitions for leg movements
 * 
 *
 *  Example Instantiation of the Module
 *   - rosrun cssr_system animateBehaviour
 *   - rosservice call /animateBehaviour/setActivation "state: 'enabled'"
 *   - rosservice call /animateBehaviour/setActivation "state: 'disabled'"
 *
 * Author: Eyerusalem Mamuye Birhan 
 * Email:  ebirhan@andrew.cmu.edu
 * Date:   2025-01-10
 * Version: 1.0
 * 
 * Copyright (C) 2023 CSSR4Africa Consortium
 * 
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 * 
 * Website: www.cssr4africa.org
 */

#include "animateBehaviour/animateBehaviourInterface.h"

int main(int argc, char **argv) {
    
    /* Initialize the ROS node and create a NodeHandle for communication. 
     * The node is named "animateBehaviour". */
    ros::init(argc, argv, "animateBehaviour");
    ros::NodeHandle nh;

    // Capture the node name and version
    nodeName = ros::this_node::getName();
    std::string softwareVersion = "Version: 1.0";

    ROS_INFO("\n%s and %s\n"
         "                  This project is funded by the African Engineering and Technology Network (Afretec)\n"
         "                  This project is funded by the African Engineering and Technology Network (Afretec)\n"
         "                  Inclusive Digital Transformation Research Grant Programme.\n"
         "                  Website: www.cssr4africa.org\n"
         "                  This program comes with ABSOLUTELY NO WARRANTY.", 
         nodeName.c_str(), softwareVersion.c_str());

   // Add signal handler
    signal(SIGINT, sigintHandler);   // Ctrl+C
    signal(SIGTERM, sigintHandler);  // Termination request
    signal(SIGABRT, sigintHandler);  // Abort signal
    signal(SIGQUIT, sigintHandler);  // Quit program
  
    // Node started up message
    ROS_INFO("%s: start-up.", nodeName.c_str());

    /* Initialize logging */
    std::string logFilePath = ros::package::getPath(ROS_PACKAGE_NAME) + "/animateBehaviour/data/animateBehaviourLogFile.log";
    std::remove(logFilePath.c_str());
    logFile.open(logFilePath, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        ROS_ERROR("[%s] Critical Error: Failed to open log file at: %s", nodeName.c_str(), logFilePath.c_str());
        return -1;
    }
    
    
    /* Start log file cleanup thread */
    std::thread closeLogFileThread([]() {
        std::this_thread::sleep_for(std::chrono::minutes(20));
        closeLogFile();
    });
    closeLogFileThread.detach();

    /* Load configuration */
    std::string configFilename = "animateBehaviourConfiguration.ini";
    std::string configPath = ros::package::getPath(ROS_PACKAGE_NAME) + "/animateBehaviour/config/" + configFilename;
    try {
        loadConfiguration(configPath);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] Configuration Error: Failed to load configuration from %s: %s", 
                  nodeName.c_str(), configPath.c_str(), e.what());
        return -1;
    }

    std::string platform;                               
    std::string behaviour;                             
    bool wasActive = false;                           
    ROS_INFO("%s: Configuration parameters:\n"
        "                                                       platform: %s\n"
        "                                                       behaviour: %s\n"
        "                                                       simulatorTopics: %s\n"
        "                                                       robotTopics: %s\n"
        "                                                       verboseMode: %s\n"
        "                                                       rotMaximumRange: %s\n"
        "                                                       selectedRange: %s\n"
        "                                                       armMaximumRange: %s\n"
        "                                                       handMaximumRange: %s\n"
        "                                                       legMaximumRange: %s\n"
        "                                                       gestureDuration: %s\n"
        "                                                       numPoints: %s\n"
        "                                                       numPointsLeg: %s\n"
        "                                                       legRepeatFactor: %s\n",
        nodeName.c_str(),
        configParams.at("platform").c_str(),
        configParams.at("behaviour").c_str(),
        configParams.at("simulatorTopics").c_str(),
        configParams.at("robotTopics").c_str(),
        configParams.at("verboseMode").c_str(),
        configParams.at("rotMaximumRange").c_str(),
        configParams.at("selectedRange").c_str(),
        configParams.at("armMaximumRange").c_str(),
        configParams.at("handMaximumRange").c_str(),
        configParams.at("legMaximumRange").c_str(),
        configParams.at("gestureDuration").c_str(),
        configParams.at("numPoints").c_str(),
        configParams.at("numPointsLeg").c_str(),
        configParams.at("legRepeatFactor").c_str());

    /* Initialize ROS service */
    ros::ServiceServer service = nh.advertiseService("animateBehaviour/setActivation", setActivation);
    ROS_INFO("%s: Service server '/animateBehaviour/setActivation' is ready and listening for requests", nodeName.c_str());
    logToFile("animateBehaviour node started. Waiting for activation...");
    ros::Rate loopRate(10);
     
     /* Start heartbeat message */
     startHeartbeat();
    
    

    /* Main control loop */
    while (ros::ok()) {
        try {
            if (!ros::ok()) {
                isActive = false;
                cleanupAndCancelGoals(nh);
                ros::Duration(0.5).sleep();
                break;
            }

            /* Active state handling */
            if (isActive) {
                if (!wasActive) {
                    logToFile("animateBehaviour node is active. Ready to animate.");

                    try {
                        /* Load and validate configuration */
                        if (verboseMode) {
                            ROS_INFO("Loading configuration from: %s", configPath.c_str());
                        }
                        
                        /* Initialize platform-specific components */
                        platform = configParams["platform"];
                        if (platform.empty()) {
                            ROS_ERROR("[%s] Platform Error: Empty platform configuration detected", nodeName.c_str());
                            logToFile("Failed to load platform configuration.");
                            isActive = false;
                            cleanupAndCancelGoals(nh);
                            continue;
                        }

                        behaviour = configParams["behaviour"];
                        loadDataBasedOnPlatform(platform);
                        logToFile("Platform configuration loaded successfully: " + platform);
                        logToFile("Configuration loaded and data initialized.");
                        
                    } catch (const std::exception &e) {
                        ROS_ERROR("[%s] Platform Error: Failed to initialize platform: %s", 
                                nodeName.c_str(), e.what());
                        logToFile("Exception caught: " + std::string(e.what()));
                        isActive = false;
                        cleanupAndCancelGoals(nh);
                        continue;
                    }
                    wasActive = true;
                }

                /* Check isActive again before executing animation */
                if (isActive) {
                    try {
                        animateBehaviour(behaviour, nh);
                    } catch (const std::exception &e) {
                       ROS_ERROR("[%s] Animation Error: Failed to execute animation behavior: %s", 
                                nodeName.c_str(), e.what());
                        isActive = false;
                        cleanupAndCancelGoals(nh);
                        continue;
                    }
                }
            } else {
                /* Handle deactivation transition */
                if (wasActive) {
                    cleanupAndCancelGoals(nh);
                    logToFile("animateBehaviour node is inactive. Activate to start animating.");
                    resetAnimateBehaviour();
                    wasActive = false;
                }
            }

            ros::spinOnce();
            loopRate.sleep();
        } catch (const std::exception &e) {
            ROS_ERROR("[%s] System Error: Unexpected error in main control loop: %s", 
                     nodeName.c_str(), e.what());
            isActive = false;
            cleanupAndCancelGoals(nh);
            wasActive = false;
        }
    }
    
     /* Cleanup */
    stopHeartbeat();
    ROS_INFO("Animate Behaviour Node: Shutting down");

    return 0;
}
