/* gestureExecutionStub.cpp
*
* Author: Adedayo Akinade
* Date: January 10, 2025
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>              // For the action client
#include <sensor_msgs/JointState.h>                             // For subscribing to the /sensor_msgs/joint_states topic
#include <control_msgs/FollowJointTrajectoryAction.h>           // For the FollowJointTrajectoryAction action
#include <string>
#include <boost/algorithm/string.hpp>                           // For string manipulation
#include <signal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Pose2D.h>                       // Include for the Pose2D message of the /robotLocalization/pose topic
#include "unit_tests/setMode.h"                               // For advertising the /overAttention/set_mode service

#define ROS

#define SOFTWARE_VERSION            "v1.0"

#define INITIALIZATION_INFO_PERIOD 5.0
#define OPERATION_INFO_PERIOD 10.0

#define ATTENTION_DISABLED_STATE "disabled"                              // The state for the disabled mode
#define ATTENTION_LOCATION_STATE "location"                              // The state for the location mode
#define ATTENTION_MODE_DISABLED 0                                       // The attention mode disabled
#define ATTENTION_MODE_LOCATION     4


// Constants for the indices of the joints in the joint state message
#define HEAD_PITCH_IDX 0
#define HEAD_YAW_IDX 1
#define HIP_PITCH_IDX 2
#define HIP_ROLL_IDX 3
#define KNEE_PITCH_IDX 4

// Constants for the head joint angles limits in radians
#define MIN_HEAD_YAW            -2.0857
#define MAX_HEAD_YAW             2.0857
#define MIN_HEAD_PITCH          -0.7068
#define MAX_HEAD_PITCH           0.6371
#define MIN_HEAD_PITCH_SCANNING -0.3
#define MAX_HEAD_PITCH_SCANNING  0.0
#define MIN_HEAD_YAW_SCANNING   -0.58353
#define MAX_HEAD_YAW_SCANNING    0.58353

// Constants for the head joint angles in radians
#define DEFAULT_HEAD_PITCH        -0.2
#define DEFAULT_HEAD_YAW           0.0

// Type definitions for the control client
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

using namespace std;
using namespace boost::algorithm;


/*  --------------------------------------------------
            GLOBAL VARIABLES
            Defined and initialized in overtAttentionImplementation.cpp 
    -------------------------------------------------- 
*/

// Joint states of the robot - updated by subscribing to the /sensor_msgs/joint_states topic
std::vector<double> head_joint_states = {0.0, 0.0};                     // Head pitch and yaw

// Coordinates of the robot in the world (x, y, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};                       // x, y, theta


// Variables for the attention mode set
int attention_mode = ATTENTION_MODE_DISABLED;                           // Stores the attention mode currently set. Default is DISABLED on initialization
double location_x = 0.0;                                                // Stores the x-coordinate of the location to pay attention to
double location_y = 0.0;                                                // Stores the y-coordinate of the location to pay attention to
double location_z = 0.0;                                                // Stores the z-coordinate of the location to pay attention to
bool location_attended_to = false;                                      // Stores the status if the location request has been attended to once
bool disabled_once = false;                                             // Stores the status if the disabled mode has been called at least once

string node_name;                                                       // The name of the ROS node
bool node_initialized = false;                                          // The node initialized flag

string topics_filename;                                                 // The topics filename
string platform;                                                        // The platform to run the gesture execution stub for
bool verbose_mode;                                                      // The verbose mode for the gesture execution stub




/*  --------------------------------------------------
            CALLBACK FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Callback function for the joint states message received from the /joint_states topic
 *   The function receives the joint states message and stores the states of the head joints
 */
void joint_states_message_received(const sensor_msgs::JointState& msg) {
    // Create an iterator for the head pitch and head yaw joints
    auto head_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadPitch");
    auto head_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadYaw");

    // Get the index of the head pitch and head yaw joints
    int head_pitch_index = std::distance(msg.name.begin(), head_pitch_iterator);
    int head_yaw_index = std::distance(msg.name.begin(), head_yaw_iterator);

    // Update the head joint states
    head_joint_states[0] = msg.position[head_pitch_index];
    head_joint_states[1] = msg.position[head_yaw_index];
}

/* 
 *   Callback function for the robot pose message received from the /robotLocalization/pose topic
 *   The function receives the robot pose message and stores the pose of the robot
 */
void robot_pose_message_received(const geometry_msgs::Pose2D& msg) {
    robot_pose[0] = msg.x;
    robot_pose[1] = msg.y;
    robot_pose[2] = msg.theta;
}

/* 
 *   Callback function for the set_activation service
 *   The function receives a request to set the activation status of the attention system 
 *   and sets the system to the specified status
 */
bool set_mode(unit_tests::setMode::Request  &service_request, unit_tests::setMode::Response &service_response){
    // Extract request parameters
    string attention_system_state = service_request.state;
    double point_location_x = service_request.location_x;
    double point_location_y = service_request.location_y;
    double point_location_z = service_request.location_z;

    // Set the attention mode
    if(attention_system_state == ATTENTION_DISABLED_STATE){
        attention_mode = ATTENTION_MODE_DISABLED;
        disabled_once = false;                                          // Set the disabled mode called status
        service_response.mode_set_success = 1;                          // Attention mode set to disabled successfully
    }
    else if(attention_system_state == ATTENTION_LOCATION_STATE){
        attention_mode = ATTENTION_MODE_LOCATION;
        location_x = point_location_x;
        location_y = point_location_y;
        location_z = point_location_z;
        location_attended_to = false;                                   // Reset the location attended to status
        service_response.mode_set_success = 1;                          // Attention mode set to location successfully
    }
    else{
        ROS_ERROR("%s: Invalid attention system state; supported states are 'disabled', and 'location'", node_name.c_str());
        service_response.mode_set_success = 0;                          // Attention mode not set to location
        attention_mode = ATTENTION_MODE_DISABLED;                        // Disable the attention mode
        disabled_once = false; 
    }

    return true;
}


/*  
 *  Function to read the gesture execution test input file
 *
 *  The function reads the gesture execution test input file and sets the variables for the gesture execution tests
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 *      topics_filename: the topics filename
 *      debug: the debug mode
 * 
 *  @return
 *      0 if the gesture execution test input file is read successfully
 *      1 if the gesture execution test input file is not read successfully
 */
int read_gesture_execution_test_configuration(string* platform, string* topics_filename, bool *debug) {
    std::string data_directory;                                      // data directory
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_directory = "/gestureExecutionTest/config/";
    data_path += data_directory;
    data_path_and_file = data_path;
    data_path_and_file += "gestureExecutionTestConfiguration.ini";

    // Open input file
    std::ifstream input_file(data_path_and_file.c_str());
    if (!input_file.is_open()){
        ROS_ERROR("%s: Failed to open the gesture execution test configuration file '%s'", node_name.c_str(), data_path_and_file.c_str());
        return 1;
    }

    else{
        std::string input_line_read;  // variable to read the line in the file
    
        std::string param_key, param_value;

        // Set the platform value to robot by default
        *platform = "robot";

        // Get key-value pairs from the input file
        while(std::getline(input_file, input_line_read)){
            std::istringstream input_line_stream(input_line_read);
            input_line_stream >> param_key >> param_value;
            trim(param_key);
            trim(param_value);

            // convert the key and value to lower case
            boost::algorithm::to_lower(param_value);
            
            // Set the platform value -- Removed from configuration file, set to robot by default but can be added if necessrary
            if(param_key == "platform"){
                *platform = param_value;
                if(*platform != "robot" && *platform != "simulator"){
                    ROS_ERROR("%s: incorrect platform value in the gesture execution test configuration file '%s'", node_name.c_str(), data_path_and_file.c_str());
                    return 1;
                }
            }

            else if(param_key == "verboseMode"){
                if(param_value == "true"){
                    *debug = true;
                }
                else{
                    *debug = false;
                }
            }
        
        }
        input_file.close();

        if(*platform == "robot"){
            *topics_filename = "pepperTopics.dat";
        }
        else if(*platform == "simulator"){
            *topics_filename = "simulatorTopics.dat";
        }
    }

    return 0;    
}


/* 
 *   Function to verify if a topic is available
 * @param:
 *   topic_name: string to store the topic name
 * 
 * @return:
 *  boolean indicating if the topic is available
 */
bool is_topic_available(std::string topic_name){
    bool topic_available = false;                                       // boolean to store if the topic is available
    ros::master::V_TopicInfo master_topics;                             // vector to store the topics
    ros::master::getTopics(master_topics);                              // get the topics

    // Iterate through the topics to check if the topic is available
    for (const auto& topic : master_topics){
        if (topic.name == topic_name){                                  // if the topic is found
            topic_available = true;                                     // set the topic as available
            break;
        }
    }

    return topic_available;                                             // return the topic availability
}

/*  
 *   Function to extract the topic from the topics file
 *   The function reads the topics file and extracts the topic for the specified key.
 *
 *   @param:
 *       key: the key to search for in the topics file
 *       topic_file_name: the topics filename
 *       topic_value: the topic value
 *
 *   @return:
 *       0 if the topic is extracted successfully
 *       1 if the topic is not extracted successfully
 */
int extract_topic(string key, string topic_file_name, string* topic_value){
    bool debug = false;                                                 // used to turn debug message on
    
    std::string topic_path;                                             // topic filename path
    std::string topic_path_and_file;                                    // topic with path and file 

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/gestureExecutionTest/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("%s: Unable to open the topic file %s", node_name.c_str(), topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;                                        // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topic_line_read)){
        std::istringstream iss(topic_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        if (param_key == key) {                                         // if the key is found
            *topic_value = param_value;                                 // set the topic value
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (*topic_value == ""){
        ROS_ERROR("%s: unable to find a valid topic for '%s'. Please check the topics file:\n\t\t\t\t\t\t '%s'.", node_name.c_str(), key.c_str(), topic_path_and_file.c_str());
        return 1;
    }
    return 0;
}


/*  --------------------------------------------------
            ACTUATOR CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/*  
 *   Function to create a control client
 *   The function creates a control client for the specified topic.
 *
 *   @param:
 *       topic_name: the topic name
 *
 *   @return:
 *       the control client
 */
ControlClientPtr create_client(const std::string& topic_name) {
    // Create a new action client
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 5;                                            // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(1.0))) {
            return actionClient;                                        // return the action client if the server is available
        }
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD,"%s: waiting for the %s controller to come up", node_name.c_str(), topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    ROS_ERROR("%s: error creating action client for %s controller: Server not available", node_name.c_str(), topic_name.c_str());
    return nullptr;                                                     // return nullptr if the server is not available
}

/* 
 *   Function to convert degrees to radians
 *   This function converts the angle in degrees to radians
 *
 * @param:
 *   degrees: the angle in degrees
 *
 * @return:
 *   the angle in radians
 */
double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI);        // David Vernon ... cast to float
    return radians;
}
 

/*  
 *   Function to rotate the robot by a specified angle in degrees.
 *   The robot is rotated by the specified angle in degrees.
 *
 *   @param:
 *       angle_degrees: the angle in degrees
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       None
 */
void rotate_robot(double angle_degrees, ros::Publisher velocity_publisher, bool debug){
    double angle_radians;                                               // stores the angle in radians
    angle_radians = radians(angle_degrees);                             // Convert angle from degrees to radians (function found in pepperKinematicsUtilities.h)

    // Declare a geometry_msgs::Twist message to send velocity commands to the robot
    geometry_msgs::Twist velocity_command;

    // Set publishing rate to 10 Hz
    ros::Rate loop_rate(10);

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Publish the velocity command to the robot
    velocity_publisher.publish(velocity_command);

    // Sleep for the duration of the rotation
    ros::Duration(4).sleep();
}

/*  
 *   Function to compute the trajectory for an actuator from a start position to an end position
 *   The function uses the minimum-jerk model of biological motion to compute the trajectory
 *
 *   @param:
 *       start_position: vector containing the start position (joint angles) of the actuator
 *       end_position: vector containing the end position (joint angles) of the actuator
 *       number_of_joints: the number of joints in the actuator
 *       trajectory_duration: the duration of the trajectory
 *       positions: vector to store the positions of the computed trajectory
 *       velocities: vector to store the velocities of the computed trajectory
 *       accelerations: vector to store the accelerations of the computed trajectory
 *       durations: vector to store the durations of the computed trajectory
 *
 *   @return:
 *       None
 */
void compute_trajectory(std::vector<double> start_position, std::vector<double> end_position, 
                        int number_of_joints, double trajectory_duration, 
                        std::vector<std::vector<double>>& positions, std::vector<std::vector<double>>& velocities, 
                        std::vector<std::vector<double>>& accelerations, std::vector<double>& durations){
    // Declare variables
    double time_t = 0;                                                  // stores the instantaneous time of the trajectory
    std::vector<double> positions_t;                                    // vector to store the positions of the trajectory
    std::vector<double> velocities_t;                                   // vector to store the velocities of the trajectory
    std::vector<double> accelerations_t;                                // vector to store the accelerations of the trajectory
    std::vector<double> duration_t;                                     // vector to store the duration of the trajectory
    double acceleration;                                                // stores the acceleration
    double velocity;                                                    // stores the velocity
    double position;                                                    // stores the position
    double time_step = 0.1;                                             // Time step between each point in the trajectory

    // Clear the existing values
    for(int i = 0; i < positions.size(); i++){
        positions[i].clear();
        velocities[i].clear();
        accelerations[i].clear();
    }
    positions.clear();
    velocities.clear();
    accelerations.clear();

    // Compute the trajectory for each point in time
    while(time_t < trajectory_duration){
        for(int i = 0; i < number_of_joints; i++){                      // Create a trajectory for each joint (5 joints for the arm)
            position = start_position[i] + (end_position[i] - start_position[i]) * ((10 * (pow(time_t/trajectory_duration, 3))) - (15 * (pow(time_t/trajectory_duration, 4))) + (6 * (pow(time_t/trajectory_duration, 5))));
            positions_t.push_back(position);

            velocity = ((end_position[i] - start_position[i])/trajectory_duration) * ((30 * (pow(time_t/trajectory_duration, 2))) - (60 * (pow(time_t/trajectory_duration, 3))) + (30 * (pow(time_t/trajectory_duration, 4))));
            velocities_t.push_back(velocity);

            acceleration = ((end_position[i] - start_position[i])/(trajectory_duration*trajectory_duration)) * ((60 * (pow(time_t/trajectory_duration, 1))) - (180 * (pow(time_t/trajectory_duration, 2))) + (120 * (pow(time_t/trajectory_duration, 3))));
            accelerations_t.push_back(acceleration);
        }
        // Store the computed trajectory
        positions.push_back(positions_t);
        velocities.push_back(velocities_t);
        accelerations.push_back(accelerations_t);
        durations.push_back(time_t);

        // Increment the time
        time_t = time_t + time_step;

        // Clear the vectors for the next iteration
        positions_t.clear();
        velocities_t.clear();
        accelerations_t.clear();
    }

    return;
}

/*  
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
}


/*  Function to move the arm to a position using the minimum-jerk model of biological motion
 *   The function moves the arm to the specified position using the control client
 *
 *   @param:
 *       client: the control client for the arm
 *       joint_names: vector containing the joint names of the arm
 *       duration: vector containing the duration of the movement
 *       gesture_duration: the duration of the gesture
 *       positions: vector containing the joint angles of the position to move the arm to
 *       velocities: vector containing the joint velocities of the position to move the arm to
 *       accelerations: vector containing the joint accelerations of the position to move the arm to
 *
 *   @return:
 *       None
 */
void move_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, 
                                        double gesture_duration, std::vector<double> duration, 
                                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, 
                                        std::vector<std::vector<double>> accelerations){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the arm to the specified joint names
    trajectory.points.resize(positions.size());                         // Set the number of points in the trajectory to the number of positions in the waypoints

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < positions.size(); i++){
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    // Send the goal to move the head to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(gesture_duration));             // Wait for the arm to reach the specified position
}


/*  
 *   Function to move the head to a position specified by the head pitch and head yaw angles
 *   The function moves the head to the specified position using the control client
 *
 *   @param:
 *       head_topic: the topic for the head
 *       head_pitch: the pitch angle of the head
 *       head_yaw: the yaw angle of the head
 *       gesture_duration: the duration of the gesture
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       0 if head is moved successfully
 *      -1 if head is not moved successfully
 */
int move_robot_head(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);

    if(head_client == nullptr){
        return -1;
    }
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};// Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;
    
    // Move the head to the specified position
    move_to_position(head_client, head_joint_names, gesture_duration, head_position);

    return 0;
}

/*  
 *   Function to move the head to a position specified by the head pitch and head yaw angles
 *   The function moves the head to the specified position using the minimum-jerk model of biological motion
 *
 *   @param:
 *       head_topic: the topic for the head
 *       head_pitch: the pitch angle of the head
 *       head_yaw: the yaw angle of the head
 *       gesture_duration: the duration of the gesture
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       0 if head is moved successfully
 *      -1 if head is not moved successfully
 */
int move_robot_head_biological_motion(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    if(head_client == nullptr){
        return -1;
    }

    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};// Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    // Compute the trajectory for the head movement
    compute_trajectory(head_joint_states, head_position, number_of_joints, gesture_duration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);

    // Move the head to the specified position using the minimum-jerk model of biological motion
    move_to_position_biological_motion(head_client, head_joint_names, gesture_duration, duration_t_head, positions_t_head, velocities_t_head, accelerations_t_head);

    return 0;
}

/*  --------------------------------------------------
            INVERSE KINEMATICS UTILITY FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function that returns the head angles given the head end-effector position (BottomCamera)
 *   The function calculates the head yaw and head pitch angles of the head chain
 *
 * @param:
 *   camera_x: the x position of the head end-effector
 *   camera_y: the y position of the head end-effector
 *   camera_z: the z position of the head end-effector
 *   head_yaw: the head yaw angle to be updated
 *   head_pitch: the head pitch angle to be updated
 *
 * @return:
 *   None
 */
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch){
   double link_1 = -38.0;
   double link_2 = 169.9;
   double link_3 = 93.6;
   double link_4 = 61.6;

   // theta1
   *head_yaw = atan2(camera_y, (camera_x - link_1));
   // theta2
   *head_pitch = asin((link_2 - camera_z) / sqrt(pow(link_4,2) + pow(link_3,2))) + atan(link_4/link_3);

   // Check if the calculated angles fall within Pepper's range. if not set the angles to 0
   if (isnan(*head_yaw) || *head_yaw < -2.1 || *head_yaw > 2.1){
      *head_yaw = 0.0;
   }
   if (isnan(*head_pitch) || *head_pitch < -0.71 || *head_pitch > 0.638){
      *head_pitch = 0.0;
   }
}

/*  --------------------------------------------------
            ATTENTION MODES CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/*  
 *   Function to execute the location attention
 *   The function moves the robot's head to look at the specified point in the environment.
 *   
 * @param:
 *   point_x: the x coordinate of the point to look at
 *   point_y: the y coordinate of the point to look at
 *   point_z: the z coordinate of the point to look at
 *   topics_file: the topics file
 *   debug: the debug mode
 * 
 * @return:
 *   1 if the attention is executed successfully
 */
int location_attention(float point_x, float point_y, float point_z, string topics_file, ros::Publisher velocity_publisher, bool debug){
    // Robot pose coordinates
    double robot_x      = robot_pose[0] * 1000;                         // Convert the robot's x coordinate from meters to millimeters
    double robot_y      = robot_pose[1] * 1000;                         // Convert the robot's y coordinate from meters to millimeters
    double robot_theta  = robot_pose[2];
    robot_theta         = radians(robot_theta);                         // Convert the robot's orientation from degrees to radians

    // Gesture duration in milliseconds
    double duration     = 1.0;

    // Flag to determine if the pointing coordinates are reachable
    bool pose_achievable = true;

    // Rotation angle for the robot
    double rotation_angle = 0.0;

    // Attention location
    double attention_location_x = 0.0;
    double attention_location_y = 0.0;
    double attention_location_z = 0.0;

    // location_x = point_x * 1000;                                        // Convert the x coordinate from meters to millimeters
    // location_y = point_y * 1000;                                        // Convert the y coordinate from meters to millimeters
    // location_z = point_z * 1000;                                        // Convert the z coordinate from meters to millimeters

    /* */
    /* Compute the pointing coordinates with respect to the robot pose in the environment */
    double relative_pointing_x = (point_x * 1000) - robot_x;                        // Convert the pointing coordinates from meters to millimeters
    double relative_pointing_y = (point_y * 1000) - robot_y;                        // Convert the pointing coordinates from meters to millimeters
    location_x = (relative_pointing_x * cos(-robot_theta)) - (relative_pointing_y * sin(-robot_theta));
    location_y = (relative_pointing_y * cos(-robot_theta)) + (relative_pointing_x * sin(-robot_theta));
    location_z = point_z * 1000;   

    /* Account for unreachable points in the cartesian space 
    (e.g. outside the robot's forward reach)
    Rotate the robot appropriately (by 90 degrees) if necessary */

    // Case 1: Pointing coordinates directly in front of the robot (+x direction): No rotation is needed, just choose arm
    if(location_x >= 0.0){
        pose_achievable = true;                                                     // The pointing coordinates are reachable without rotating the robot
    }
    // Case 2: Pointing coordinates directly behind the robot (-x direction): 
    // Rotate the robot by 90 degrees left or right depending on the y coordinate of the pointing coordinates
    else if(location_x < 0.0){
        pose_achievable = false;
        double temp_var = 0.0;
        // Rotate 90 degrees clockwise and use right arm if the pointing coordinates are to the right of the robot (-y direction)
        if(location_y <= 0.0){
            rotation_angle = -90.0;
            // Realign the pointing coordinates considering the rotation
            temp_var = location_x;
            location_x = -location_y;
            location_y = temp_var;
        }
        // Rotate 90 degrees anticlockwise and use left arm if the pointing coordinates are to the left of the robot (+y direction)
        else if(location_y > 0.0){
            rotation_angle = 90.0;
            // Realign the pointing coordinates considering the rotation
            temp_var = location_x;
            location_x = location_y;
            location_y = -temp_var;
        }
    }
    
    // Variables to store the head angles
    double head_yaw; 
    double head_pitch;

    get_head_angles(location_x, location_y, location_z, &head_yaw, &head_pitch);
    // 33.26, -304, 120.17
    if(debug){
        ROS_INFO("%s: Head Pitch: %f, Head Yaw: %f", node_name.c_str(), head_pitch, head_yaw);
    }

    std::string head_topic;                                             // stores the head topic
    // Extract the topic for the head
    if(extract_topic("Head", topics_file, &head_topic) != 0){
        return -1;                                                      // return -1 if the head topic is not extracted successfully
    }

    // Rotate the robot by 90 degrees if the pointing coordinates are unreachable
    if(!pose_achievable){
        rotate_robot(rotation_angle, velocity_publisher, debug);
    }

    // if(move_robot_head(head_topic, head_pitch, head_yaw, duration, debug) != 0){
    //     ROS_WARN("%s: error moving the robot's head to the specified location.", node_name.c_str());
    //         return -1;                                                      // return -1 if the robot's head is not moved to the specified location
    // }
    if(move_robot_head_biological_motion(head_topic, head_pitch, head_yaw, duration, debug) != 0){
        ROS_WARN("%s: error moving the robot's head to the specified location.", node_name.c_str());
        return -1;                                                      // return -1 if the robot's head is not moved to the specified location
    }

    return 1;                                                           // attention executed successfully
}

/*  
 *   Function to handle the shutdown signal
 *   Cancels all active goals and shuts down ROS
 *  @param:
 *      sig: the signal
 *  @return:
 *      None
 */
void shut_down_handler(int sig) {
    printf("\n");
    ROS_WARN("%s: shutting down...", node_name.c_str());

    if(node_initialized){
        // Extract the topic for the head
        std::string head_topic;     // stores the head topic
        if(extract_topic("Head", topics_filename, &head_topic) == 0){
            // Set to head to horizontal looking forward
            if(move_robot_head_biological_motion(head_topic, DEFAULT_HEAD_PITCH, DEFAULT_HEAD_YAW, 1.0, verbose_mode) != 0){
                ROS_WARN("%s: error moving the robot's head to the default location", node_name.c_str());
            }
        }
    }

    // Shutdown the node
    ROS_ERROR("%s: terminated.", node_name.c_str());
    ros::shutdown();
}


int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "gestureExecutionStub", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    int attention_execution_status = 0;

    ros::Publisher attention_velocity_publisher;                            // The publisher for the velocity commands

    node_name = ros::this_node::getName();

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                   // The signal handler for the interrupt signal    

    std::string copyright_message = node_name + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t       This project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\t\t\t       Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t       Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t       This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                  // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());

    // Read the gesture execution test configuration file
    if(read_gesture_execution_test_configuration(&platform, &topics_filename, &verbose_mode) != 0){
        ROS_ERROR("%s: error reading the gesture execution test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }

    ROS_INFO("%s: platform: %s, topics file: %s, verbose mode: %s", node_name.c_str(), platform.c_str(), topics_filename.c_str(), verbose_mode ? "true" : "false");

    // Extract the joint_states topic and subscribe to the joint states topic
    std::string joint_states_topic;
    if(extract_topic("JointStates", topics_filename, &joint_states_topic)){
        ROS_ERROR("%s: error extracting the joint states topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s...", node_name.c_str(), joint_states_topic.c_str());
    while(ros::ok() && !is_topic_available(joint_states_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), joint_states_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber joint_states_subscriber = nh.subscribe(joint_states_topic, 1, &joint_states_message_received);
    ROS_INFO("%s: subscribed to %s.", node_name.c_str(), joint_states_topic.c_str());

    // Create a publisher for the velocity commands
    std::string velocity_topic;
    if(extract_topic("Wheels", topics_filename, &velocity_topic)){
        ROS_ERROR("%s: error extracting the wheels topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: creating a publisher for the velocity commands...", node_name.c_str());
    while(ros::ok() && !is_topic_available(velocity_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), velocity_topic.c_str());
        attention_velocity_publisher = nh.advertise<geometry_msgs::Twist>(velocity_topic.c_str(), 1, true);
        ros::Duration(1).sleep();
    }
    // attention_velocity_publisher = n.advertise<geometry_msgs::Twist>(velocity_topic.c_str(), 1000, true);
    ROS_INFO("%s: created a publisher for the velocity commands.", node_name.c_str());

    // Subscribe to the /robotLocalization/pose topic
    std::string robot_pose_topic;
    if(extract_topic("RobotPose", topics_filename, &robot_pose_topic)){
        ROS_ERROR("%s: error extracting the robot pose topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to robot pose topic...", node_name.c_str());
    while(ros::ok() && !is_topic_available(robot_pose_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), robot_pose_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber robot_pose_subscriber = nh.subscribe(robot_pose_topic, 1, &robot_pose_message_received);
    ROS_INFO("%s: subscribed to robot pose topic.", node_name.c_str());


    // Advertise the /overtAttention/set_mode service
    std::string set_mode_service_name = "/overtAttention/set_mode";
    ros::ServiceServer set_mode_service;
    ROS_INFO("%s: advertising the %s service...", node_name.c_str(), set_mode_service_name.c_str());
    while(ros::ok() && !set_mode_service){
        set_mode_service = nh.advertiseService(set_mode_service_name, set_mode);
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s service ready", node_name.c_str(), set_mode_service_name.c_str());

    // Print the node ready message after initialization complete
    ROS_INFO("%s: stub ready.", node_name.c_str());

    // Extract the topic for the head
    std::string head_topic;     // stores the head topic
    if(extract_topic("Head", topics_filename, &head_topic) == 0){
        // Set to head to horizontal looking forward
        if(move_robot_head_biological_motion(head_topic, DEFAULT_HEAD_PITCH, DEFAULT_HEAD_YAW, 1.0, verbose_mode) != 0){
            ROS_ERROR("%s: error setting the head to the horizontal looking forward pose.", node_name.c_str());
            shut_down_handler(0);
            return 0;                                                                // return -1 if the head cannot be set to the horizontal looking forward pose
        }
        node_initialized = true;                                                     // Set the node initialized flag to true
    }

    while(ros::ok()){
        ROS_INFO_THROTTLE(OPERATION_INFO_PERIOD, "%s: running...", node_name.c_str());  // Print a message every 10 seconds
        ros::spinOnce();

        /* Execute the attention mode selected in the request */
        switch(attention_mode){
            case ATTENTION_MODE_DISABLED:                                       // Disabled attention mode
                if(!disabled_once){                                             // If the disabled mode has not been called at least once, set the previous time to the current time
                    // Set head to center pose looking forward
                    if(move_robot_head_biological_motion(head_topic, DEFAULT_HEAD_PITCH, DEFAULT_HEAD_YAW, 1.0, verbose_mode) != 0){
                        ROS_ERROR("%s: error setting the head to the horizontal looking forward pose in disabled mode.", node_name.c_str());
                    }
                    disabled_once = true;
                }
                break;
            case ATTENTION_MODE_LOCATION:                                       // Location attention mode
                // Check if the location has already been attended to
                if(!location_attended_to){                                      // If location has not been attended to, attend to the location
                    // Call the location attention function
                    attention_execution_status = location_attention(location_x, location_y, location_z, topics_filename, attention_velocity_publisher, verbose_mode);
                    location_attended_to = true;                                // Set the location attended to status to true
                }
                break;
            default:                                                            // Invalid attention mode
                if(verbose_mode){
                    ROS_ERROR_THROTTLE(OPERATION_INFO_PERIOD, "%s: invalid attention mode selected", node_name.c_str());
                }
                break;
        }
        
    }

    return 0;
}