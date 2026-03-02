/* animateBehaviourInterface.h
* Interface header file containing service, topics, messages, 
* functions definitions and declarations for the robot's animation behavior.
*
* Contains:
* - Global variable declarations 
* - Type definitions for ROS action clients
* - Core system function declarations
* - Movement generation function declarations
* - Joint control function declarations
* - Helper function declarations
* - State management function declarations
*
* Author: Eyerusalem Mamuye Birhan
* Date:   2025-01-10
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
*/

#ifndef ANIMATE_BEHAVIOUR_INTERFACE_H
#define ANIMATE_BEHAVIOUR_INTERFACE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "cssr_system/setActivation.h"
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <atomic>
#include <random>
#include <thread>
#include <boost/thread.hpp>
#include <csignal>     
#include <signal.h> 
#include <boost/thread.hpp>
#include <boost/chrono.hpp>


// Type definitions
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

// Global variables declarations
extern bool isActive;
extern std::ofstream logFile; 
extern bool verboseMode;
extern std::string nodeName; 
extern std::map<std::string, std::string> configParams;
extern std::map<std::string, std::string> topicData;
extern std::vector<std::string> rArmJointNames;
extern std::vector<std::string> lArmJointNames;
extern std::vector<std::string> rhandJointNames;
extern std::vector<std::string> lhandJointNames;
extern std::vector<std::string> legJointNames;
extern std::ofstream logFile;
extern const std::string NODE_NAME;
extern boost::thread* heartbeatThread;
extern std::atomic<bool> shouldRunHeartbeat;
extern const std::string FLAG_FILE_PATH;
extern std::atomic<bool> shutdown_in_progress;

/**
 * @brief Initializes the ROS node and sets up required components
 * 
 * This function handles the complete initialization sequence including:
 * - Displaying startup messages
 * - Initializing the log file
 * - Loading configuration
 * - Setting up initial parameters
 * 
 * @throws std::runtime_error If initialization fails
 */
void initializeNode();

/**
 * @brief Executes the heartbeat message loop
 * 
 * Runs in a separate thread and periodically outputs node status.
 * Continues until shouldRunHeartbeat is false or ROS shutdown.
 * Outputs status message every 5 seconds.
 */
void runHeartbeat();

/**
 * @brief Initiates the heartbeat system
 * 
 * Creates and starts a new thread running the heartbeat loop.
 * Sets up the shouldRunHeartbeat flag and initializes the thread.
 * Should be called once during node startup.
 */
void startHeartbeat();

/**
 * @brief Safely stops the heartbeat system
 * 
 * Stops the heartbeat thread by:
 * - Setting shouldRunHeartbeat to false
 * - Waiting for thread completion
 * - Cleaning up thread resources
 * Should be called during node shutdown.
 */
void stopHeartbeat();

/**
 * @brief Performs cleanup operations during node shutdown
 * 
 * Handles graceful shutdown including:
 * - Stopping the heartbeat
 * - Closing log files
 * - Cleaning up resources
 * - Outputting final status messages
 */
void cleanupNode();

/**
 * @brief Logs a message to the log file
 * @param message The message to log
 * 
 */
void logToFile(const std::string &message);

/**
 * Closes the log file safely
 */
void closeLogFile();


/**
 * Resets the animate behaviour by removing the flag file
 */
void resetAnimateBehaviour();


/* Add after the global variables, before other functions */

/**
 * @brief Cancels all active movement goals and stops the robot
 * @param nh ROS NodeHandle for communication
 */
void cleanupAndCancelGoals(ros::NodeHandle& nh);

/* Add after cleanupAndCancelGoals function */

/**
 * @brief Handles shutdown signal (Ctrl+C)
 * @param sig Signal number
 */
void sigintHandler(int sig);

// Core functionality
/**
 * @brief Loads and parses configuration from specified file.
 * @param filename Path to configuration file
 * @return None
 * 
 * Reads key-value pairs from file, populates configParams map.
 * Sets verboseMode and validates required parameters.
 */
void loadConfiguration(const std::string& filename);

/**
 * @brief Loads platform-specific topic mapping data from file.
 * @param platform Target platform ("robot" or "simulator")
 */
void loadDataBasedOnPlatform(const std::string& platform);

/**
 * ROS service handler for enabling/disabling the animate behaviour
 * @param req Service request containing desired state
 * @param res Service response indicating success/failure ("1" for enabled, "0" for disabled)
 * @return Always returns true to indicate service was handled
 */
bool setActivation(cssr_system::setActivation::Request &req, cssr_system::setActivation::Response &res);

/**
 * @brief Executes animation behaviors based on the given behavior parameter.
 * 
 * This function activates subtle body movements, hand flex movements, or base rotation
 * depending on the input. If no specific behavior is requested, it performs all actions.
 * 
 * @param behaviour A string indicating the desired animation behavior(s).
 * @param nh The ROS NodeHandle to manage communication with ROS.
 */
void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh);

// Movement functions
/**
 * @brief Executes flexible movements for the robot's hands and wrists.
 * 
 * This function controls the left and right arms and hands for the robot platform,
 * and only the arms for the simulator platform. It launches threads for simultaneous
 * control of the limbs based on the specified movement type.
 * 
 * @param nh The ROS NodeHandle to manage communication with ROS.
 * @param movementType A string indicating the type of movement ("flexi" or "All").
 */
void flexiMovement(ros::NodeHandle& nh, const std::string& movementType);

/**
* @brief Executes subtle body movements for the entire robot.
* 
* This function controls and coordinates movements of multiple robot parts 
* including arms, hands, and leg joints to create natural-looking animations.
* For the physical robot, it controls left/right arms, hands, and leg movements.
* For the simulator, it controls only the arms and leg movements (no hands).
* All movements are executed simultaneously using separate threads.
*
* @param nh The ROS NodeHandle to manage communication with ROS.
*/
void subtleBodyMovement(ros::NodeHandle& nh);

/**
 * @brief Initiates a rotation of the robot base using a calculated angular velocity.
 * 
 * This function performs a sequence of rotations with angular velocities calculated
 * using the calculateAngularVelocityZ function. It alternates between positive and
 * negative velocities with stops in between, completing a specified number of cycles.
 * 
 * @param nh The ROS NodeHandle.
 */
void rotationBaseShift(ros::NodeHandle& nh);

// Joint-specific movement functions
/**
 * @brief Controls animated movement of robot's right arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param rightArmTopic Topic for right arm control
 * @param resetPosition Flag to reset to home position
 */
void rArm(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition);

/**
 * @brief Controls animated movement of robot's left arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftArmTopic Topic for left arm control
 * @param resetPosition Flag to reset to home position
 */
void lArm(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition);

/**
 * @brief Animates the right hand of the robot.
 * 
 * This function calculates target positions for the right hand joints and moves the hand
 * to these positions using a biological trajectory. It also generates and displays diagnostic 
 * information if verbose mode is enabled. The function supports resetting to the home position
 * if requested.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param rightHandTopic The ROS topic name for controlling the right hand's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void rHand(ros::NodeHandle& nh, std::string rightHandTopic, bool resetPosition);

/**
 * @brief Controls animated movement of robot's left hand.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftHandTopic Topic for left hand control
 * @param resetPosition Flag to reset to home position
 */
void lHand(ros::NodeHandle& nh, std::string leftHandTopic, bool resetPosition);


/**
 * @brief Animates the leg of the robot.
 * 
 * This function calculates target positions for the leg joints and moves the leg
 * to the calculated positions. It first moves the leg to the home position if it is the
 * first run or if a reset is explicitly requested. After that, it continuously moves the leg
 * through randomly generated positions within the defined joint limits. The first move
 * is handled using the moveToPosition function, and subsequent moves are performed using
 * the moveToPositionBiological function for smooth and natural movement.
 * 
 * Diagnostic information is logged if verbose mode is enabled, including details of
 * each movement's joint positions and target positions. The function supports resetting
 * the leg's position to a home state and allows continuous movement from the last position.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param legTopic The ROS topic name for controlling the leg's movement.
 * @param resetPosition Boolean flag indicating whether to reset the leg to the home position.
 */
void leg(ros::NodeHandle& nh, std::string legTopic, bool resetPosition);

/**
 * @brief Controls flexible wrist movement of robot's right arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param rightArmTopic Topic for right arm control
 * @param resetPosition Flag to reset to home position
 */
void rArml(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition);

/**
 * @brief Controls flexible movement of robot's left arm wrist.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftArmTopic Topic for left arm control
 * @param resetPosition Flag to reset to home position
 */
void lArml(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition);

// Helper functions
/**
 * @brief Creates action client with exponential backoff connection.
 * @param topicName ROS topic name for action client
 * @return Pointer to created action client
 * @throws std::runtime_error if connection fails after max attempts
 */
ControlClientPtr createClient(const std::string& topicName);

/**
 * @brief Moves robot to specified position using action server.
 * 
 * @param client Action client for sending goals
 * @param jointNames Names of joints to move
 * @param positionName Identifier for target position
 * @param positions Target joint positions
 */
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames,
                   const std::string& positionName, std::vector<double> positions);

/**
 * @brief Executes biologically-inspired joint movement trajectory.
 * 
 * @param client Action client for sending goals
 * @param jointNames Names of joints to control
 * @param positionName Target position identifier
 * @param positions Vector of target positions for each joint
 * @param durations Duration for each trajectory point
 */
void moveToPositionBiological(ControlClientPtr& client, 
                            const std::vector<std::string>& jointNames,
                            const std::string& positionName, 
                            const std::vector<std::vector<double>>& positions, 
                            const std::vector<double>& durations);

/**
 * @brief Generates random position within specified range.
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return Random value between min and max
 */
double generateRandomPosition(double min, double max);

/**
 * @brief Parse comma-separated percentage values into vector.
 * @param percentagesStr String of comma-separated percentages
 * @return Vector of parsed percentage values
 */
std::vector<double> parsePercentages(const std::string& percentagesStr);

/**
 * @brief Calculates an angular velocity in the Z-axis for the robot's base rotation.
 * 
 * This function calculates a target angular velocity within specified ranges. It ensures the
 * angular velocity is within the max and min positions and adjusts based on the 'rotMaximumRange' 
 * and 'selectedRange' from configParams.
 * 
 * @param maxAngularVelocity The maximum allowable angular velocity.
 * @return The calculated angular velocity in the Z-axis.
 */
double calculateAngularVelocityZ(double maxAngularVelocity);

/**
 * @brief Calculates target positions for robot movement.
 * 
 * @param homePosition Current/home position of robot joints
 * @param maxPosition Maximum allowable position for joints
 * @param minPosition Minimum allowable position for joints
 * @param jointType Joint type ("arm", "leg", or "hand")
 * @param count Number of positions to generate
 * @return Vector of calculated target positions
 */
std::vector<std::vector<double>> calculateTargetPosition(const std::vector<double>& homePosition,
                                                       const std::vector<double>& maxPosition,
                                                       const std::vector<double>& minPosition,
                                                       const std::string& jointType,
                                                       int count);

// State management
/**
 * @brief Check if this is the first run of the program.
 * @return true if first run, false otherwise
 */
bool isFirstRun();

/**
 * @brief Update flag file to indicate program has run.
 */
void updateFirstRunFlag();

/**
 * Resets the animate behaviour by removing the flag file
 */
void resetAnimateBehaviour();

#endif // ANIMATEBEHAVIOURCONFIGANDSERVICE_H