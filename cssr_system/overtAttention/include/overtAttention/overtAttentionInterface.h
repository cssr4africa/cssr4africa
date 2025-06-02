/* overtAttentionInterface.h
*
* Author: Muhammed Danso and Adedayo Akinade
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

#ifndef OVERT_ATTENTION_INTERFACE_H
#define OVERT_ATTENTION_INTERFACE_H

// Include the necessary libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <actionlib/client/simple_action_client.h>              // For the action client
#include <sensor_msgs/JointState.h>                             // For subscribing to the /sensor_msgs/joint_states topic
#include <control_msgs/FollowJointTrajectoryAction.h>           // For the FollowJointTrajectoryAction action
#include <string>
#include <boost/algorithm/string.hpp>                           // For string manipulation
#include <geometry_msgs/Twist.h>                                // For publishing Twist messages
#include <geometry_msgs/PoseStamped.h>                          // For storing the pose of the robot
#include <vector>
#include <random>
#include <chrono>
#include <string.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <complex>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <signal.h>
#include <csignal>
#include <std_msgs/Float64.h>                                   // For publishing Float64 messages
#include "std_msgs/Float32.h"                                   // For publishing Float32 messages
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "cssr_system/face_detection_msg_file.h"                            // For subscribing to the /face_detection/data topic
#include <geometry_msgs/Pose2D.h>                       // Include for the Pose2D message of the /robotLocalization/pose topic
#include "geometry_msgs/Point.h"                                // For storing the centroids of the detected faces
#include "cssr_system/setMode.h"                               // For advertising the /overAttention/set_mode service
#include "cssr_system/Status.h"                                   // For publishing /overtAttention/mode messages
#include <iomanip> 
#include <regex>
#include <image_transport/image_transport.h>                    // For image transport
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <cv_bridge/cv_bridge.h>

// Namespaces used in the software
using namespace std;
using namespace cv;
using namespace boost::algorithm;

// Constants for the ROS
#define ROS
#define DEBUG 0

#define SOFTWARE_VERSION            "v1.0"
// Constants for the attention modes
#define ATTENTION_SOCIAL_STATE      "social"
#define ATTENTION_SCANNING_STATE    "scanning"
#define ATTENTION_LOCATION_STATE    "location"
#define ATTENTION_SEEKING_STATE     "seeking"
#define ATTENTION_DISABLED_STATE    "disabled"
#define ATTENTION_MODE_DEFAULT     -1
#define ATTENTION_MODE_DISABLED     0
#define ATTENTION_MODE_SOCIAL       1
#define ATTENTION_MODE_SCANNING     2
#define ATTENTION_MODE_SEEKING      3
#define ATTENTION_MODE_LOCATION     4

#define SALIENCY_SOCIAL_CONTROL     0
#define RANDOM_SOCIAL_CONTROL       1

//  Constants for the status of mutual gaze detection
#define DETECTING_MUTUAL_GAZE       1
#define MUTUAL_GAZE_DETECTED        2
#define MUTUAL_GAZE_NOT_DETECTED    3

// Constants for the activation status of the attention system
#define ATTENTION_ENABLED_STATE     "enabled"
#define ATTENTION_DISABLED_STATE    "disabled"
#define ATTENTION_SYSTEM_ENABLED    1
#define ATTENTION_SYSTEM_DISABLED   0

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

// Pepper robot links lengths (mm)
#define TORSO_HEIGHT            820.0

// Number of faces to look through if there are multiple faces
#define NUMBER_OF_FACES_SOCIAL_ATTENTION 3

// Constant for the timeout in seconds for seeking mutual gaze
#define ENGAGEMENT_TIMEOUT 12.00

// Constants for the camera in image processing
// #define VFOV                            44.30
// #define HFOV                            55.20
// #define IMG_W                           640
// #define IMG_H                           480

#define VFOV_PEPPER_FRONT_CAMERA        44.30
#define HFOV_PEPPER_FRONT_CAMERA        55.20
#define IMG_WIDTH_PEPPER_FRONT_CAMERA   640
#define IMG_HEIGHT_PEPPER_FRONT_CAMERA  480

#define VFOV_PEPPER_STEREO_CAMERA       44.30
#define HFOV_PEPPER_STEREO_CAMERA       55.20
#define IMG_WIDTH_PEPPER_STEREO_CAMERA  640
#define IMG_HEIGHT_PEPPER_STEREO_CAMERA 480

#define VFOV_REALSENSE_CAMERA           42.50
#define HFOV_REALSENSE_CAMERA           69.50
#define IMG_WIDTH_REALSENSE_CAMERA      640
#define IMG_HEIGHT_REALSENSE_CAMERA     480

// Constants for the saliency map computation
#define PATCH_RADIUS 15
#define HABITUATION_RATE 0.1
#define IOR_LIMIT 50


// Periods for printing ROS_INFO/ROS_ERROR messages during initialization and operation
#define INITIALIZATION_INFO_PERIOD          5.0
#define OPERATION_INFO_PERIOD               10.0

// Type definitions for the control client
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

/*
 *   Struct to store the angle changes required to refocus the robot head on a point in its FOV
 *   The struct stores the delta yaw and delta pitch angles required to refocus the robot head on a point in its FOV
 */
struct AngleChange {
    double delta_yaw;
    double delta_pitch;
};

/*
 *   Struct to store the pixel coordinates of a point in the image
 *   The struct stores the x and y pixel coordinates of a point in the image
 */
struct PixelCoordinates {
    int x;
    int y;
};

/*  --------------------------------------------------
            GLOBAL VARIABLES
            Defined and initialized in overtAttentionImplementation.cpp 
    -------------------------------------------------- 
*/

// Joint states of the robot - updated by subscribing to the /sensor_msgs/joint_states topic
extern std::vector<double> head_joint_states;

// Coordinates of the robot in the world (x, y, theta) - updated by subscribing to the /robotLocalization/pose topic
extern std::vector<double> robot_pose;

// Head joint angles for the robot control during scanning or social attention
extern std::vector<double> attention_head_pitch;
extern std::vector<double> attention_head_yaw;

extern double angle_of_sound;                                                   // Stores the angle of the sound source
extern double previous_angle_of_sound;                                          // Stores the previous angle of the sound source

// Variables for information about the detected faces and sound
extern bool face_detected;                                                      // Stores the status of face detection
extern bool sound_detected;                                                     // Stores the status of sound detection
extern bool mutual_gaze_detected;                                               // Stores the status of mutual gaze detection

extern std::vector<int> face_labels;                                            // Stores the labels of the detected faces
extern int last_seen_label;                                                     // Stores the label of the last seen face
extern int current_label;                                                       // Stores the label of the current face

// Variables for the attention mode set
extern int attention_mode;                                                      // Stores the attention mode currently set. Default is DISABLED on initialization
extern double location_x;                                                       // Stores the x-coordinate of the location to pay attention to
extern double location_y;                                                       // Stores the y-coordinate of the location to pay attention to
extern double location_z;                                                       // Stores the z-coordinate of the location to pay attention to
extern bool location_attended_to;                                               // Stores the status if the location request has been attended to once

extern bool social_attention_done;                                              // Flag to indicate if social attention is done

extern int sound_count;                                                         // Counter for the number of times sound has been detected

extern bool seek_once;                                                          // Variable to check if the seek mode has been called at least once

extern bool scan_once;                                                          // Variable to check if the scan mode has been called at least once

extern bool disabled_once;                                                      // Variable to check if the disabled mode has been called at least once

extern bool seeking_completed;                                                  // Flag to indicate if seeking is completed

extern size_t seeking_index;                                                    // Index for the seeking angles

extern cssr_system::Status overt_attention_mode_msg;                            // Message to publish the attention mode

// Variables for the saliency features
extern std::vector<std::tuple<double, double, int>> previous_locations;         // Stores locations that won in the WTA
extern std::vector<std::tuple<double, double, int>> face_locations;             // Stores the locations of the detected faces saliency
extern cv::Mat faces_map;                                                       // Stores the map of the faces detected
extern cv::Mat camera_image;                                                    // Hold image from the robot camera for scanning mode

// Variables for the image parameters
extern double vertical_fov;                                                     // Vertical field of view of the camera
extern double horizontal_fov;                                                   // Horizontal field of view of the camera
extern int camera_image_width;                                                  // Width of the image
extern int camera_image_height;                                                 // Height of the image

// Variables for the random number generator in  the social function
extern std::mt19937 random_generator;
extern std::uniform_int_distribution<int> random_distribution;

// Publisher for the velocity commands
extern ros::Publisher attention_velocity_publisher;

// Declare the publisher of the /overt_attention/mode topic
extern ros::Publisher overt_attention_mode_pub;

// Configuration parameters
extern std::string implementation_platform;
extern std::string camera_type;
extern int realignment_threshold;
extern int x_offset_to_head_yaw;
extern int y_offset_to_head_pitch;
extern std::string simulator_topics;
extern std::string robot_topics;
extern string topics_filename;
extern int social_attention_mode;
extern bool verbose_mode;

extern std::string node_name;                                                    // Stores the name of the node

extern bool shutdown_requested;                                                  // Flag to indicate if a shutdown has been requested
extern bool node_initialized;                                                    // Flag to indicate if the node has been initialized





/* ------ CALLBACK FUNCTIONS ------ */

/*
 *   Callback function for the face detection data received from the /face_detection/data topic
 *   The function receives the face detection data and computes the required head angles to look at the detected faces
 */
void face_detection_data_received(const cssr_system::face_detection_msg_file& data_msg);

/* 
 *   Callback function for the sound localization data received from the /soundDetection/direction topic
 *   The function receives the sound localization data and stores the angle of the sound source
 */
void sound_localization_data_received(const std_msgs::Float32& data_msg);

/* 
 *   Callback function for the camera image received from the /camera/color/image_raw topic
 *   The function receives the camera image and converts it to a cv::Mat image
 */
void front_camera_message_received(const sensor_msgs::ImageConstPtr& msg);

/* 
 *   Callback function for the joint states message received from the /joint_states topic
 *   The function receives the joint states message and stores the states of the head joints
 */
void joint_states_message_received(const sensor_msgs::JointState& msg);

/* 
 *   Callback function for the robot pose message received from the /robotLocalization/pose topic
 *   The function receives the robot pose message and stores the pose of the robot
 */
void robot_pose_message_received(const geometry_msgs::Pose2D& msg);

/* 
 *   Callback function for the set_activation service
 *   The function receives a request to set the activation status of the attention system 
 *   and sets the system to the specified status
 */
bool set_mode(cssr_system::setMode::Request  &service_request, cssr_system::setMode::Response &service_response);




/*  --------------------------------------------------
            CONFIGURATION CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input);

/* 
 *   Function to verify if a topic is available
 * @param:
 *   topic_name: string to store the topic name
 * 
 * @return:
 *  boolean indicating if the topic is available
 */
bool is_topic_available(std::string topic_name);

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
int extract_topic(string key, string topic_file_name, string* topic_value);

/* 
 *   Function to set the image parameters for the camera
 *   The function sets the image parameters for the specified camera.
 *
 *   @param:
 *       platform: the platform value
 *       camera: the camera value
 *       vertical_fov: the vertical field of view of the camera
 *       horizontal_fov: the horizontal field of view of the camera
 *       image_width: the width of the image
 *       image_height: the height of the image
 *
 *   @return:
 *       None
 */
void set_image_parameters(string platform, string camera, double* vertical_fov, double* horizontal_fov, int* image_width, int* image_height);

/* 
 *   Function to read the overt attention configuration.
 *   The configuration file contains the platform, camera, realignment threshold, x offset to head yaw, y offset to head pitch, simulator topics, robot topics, topics filename, and debug mode.
 *   The function reads the configuration file and sets the values for the specified parameters.
 * 
 * @param:
 *   platform: the platform value
 *   camera: the camera value
 *   realignment_threshold: the realignment threshold value
 *   x_offset_to_head_yaw: the x offset to head yaw value
 *   y_offset_to_head_pitch: the y offset to head pitch value
 *   simulator_topics: the simulator topics value
 *   robot_topics: the robot topics value
 *   topics_filename: the topics filename value
 *   social_attention_mode: the social attention mode value
 *   debug_mode: the debug mode value
 * 
 * @return:
 *   0 if the configuration file is read successfully
 *   1 if the configuration file is not read successfully
 */
int read_configuration_file(string* platform, string* camera, int* realignment_threshold, int* x_offset_to_head_yaw, int* y_offset_to_head_pitch, string* simulator_topics, string* robot_topics, string* topics_filename, int* social_attention_mode, bool* debug_mode);


/* 
 *   Function to print the overt attention configuration
 *
 *  @param:
 *     platform: the platform value
 *     camera: the camera value
 *     realignment_threshold: the realignment threshold value
 *     x_offset_to_head_yaw: the x offset to head yaw value
 *     y_offset_to_head_pitch: the y offset to head pitch value
 *     simulator_topics: the simulator topics value
 *     robot_topics: the robot topics value
 *     topics_filename: the topics filename value
 *     debug_mode: the debug mode value
 * 
 *  @return:
 *    None
 */
void print_configuration(string platform, string camera, int realignment_threshold, int x_offset_to_head_yaw, int y_offset_to_head_pitch, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode);




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
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch);




/*  --------------------------------------------------
            IMAGE PIXEL UTILITY FUNCTIONS 
    -------------------------------------------------- 
*/

/*  
 *   Function to compute the angle changes required refocus the robot head on a point it's FOV
 *
 *   @param:
 *       center_x: x coordinate of the point of interest
 *       center_y: y coordinate of the point of interest
 *       image_width: width of the original image
 *       image_height: height of the original image
 *       theta_v: the vertical FOV of the camera
 *       theta_h: the horizontal FOV of the camera
 *
 *   @return:
 *       AngleChange: the head_yaw and head_pitch angle changes required
 */

AngleChange get_angles_from_pixel(double center_x, double center_y, double image_width, double image_height, double theta_h, double theta_v);

/*  
 *   Function to compute the image pixel coordinates for points in the world given the required change of angle from the current head pose.
 *
 *   @param:
 *       delta_yaw (float): Change in yaw angle (in radians)
 *       delta_pitch (float): Change in pitch angle (in radians)
 *       W (int): Width of the image in pixels
 *       H (int): Height of the image in pixels
 *       theta_h (float): Horizontal field of view of the camera (in degrees)
 *       theta_v (float): Vertical field of view of the camera (in degrees)
 *
 *   @return:
 *       (x, y): Pixel coordinates corresponding to the angle changes
 */
PixelCoordinates calculate_pixel_coordinates(double delta_yaw, double delta_pitch, int W, int H, double theta_h, double theta_v);





/*  --------------------------------------------------
            SALEIENCY COMPUTATION FUNCTIONS 
    -------------------------------------------------- 
*/

/*  
 *   Function to compute the most salient point in a saliency map.
 *
 *   @param:
 *       saliencyMap (matrix): the map of points with values indicating saliency in increasing order
 *
 *   @return:
 *       (x, y): Pixel coordinates corresponding to the winner
 */
std::pair<int, int> winner_takes_all(const cv::Mat& saliency_map);

/*  
 *   Function to gradually reduce the saliency of previously attended points.
 *
 *   @param:
 *       saliencyMap (matrix): the map of points with values indicating saliency in increasing order
 *       wtaMap (matrix): a white background image showing all previous attended points in some variation of blue
 *       previous_locations (vector): the list of previous attended points
 *
 *   @return:
 *       wtaMap (matrix): a white background image showing all previous attended points in some variation of blue
 *       previous_locations (vector): the new updated list of previous attended points
 */
std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> habituation(cv::Mat& saliency_map, cv::Mat& wta_map, const std::vector<std::tuple<double, double, int>>& previous_locations);

/*  
 *   Function to inhibit previous attended points that have reached the IOR limit and remove them from previous attended locations.
 *
 *   @param:
 *       saliencyMap (matrix): the map of points with values indicating saliency in increasing order
 *       wtaMap (matrix): a white background image showing all previous attended points in some variation of blue
 *       previous_locations (vector): the list of previous attended points
 *
 *   @return:
 *       wtaMap (matrix): a white background image showing all previous attended points in some variation of blue
 *       previous_locations (vector): the new updated list of previous attended points
 */
std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> inhibition_of_return(cv::Mat& saliency_map, cv::Mat& wta_map, const std::vector<std::tuple<double, double, int>>& previous_locations);

/*  
 *   Function to compute the saliency features of an image.
 *
 *   @param:
 *       camera_image (matrix): the image to compute the saliency features
 *       centre_x (int): the x coordinate of the most salient point
 *       centre_y (int): the y coordinate of the most salient point
 *       debug_mode (bool): the debug mode
 *
 *   @return:
 *       0 if the saliency features are computed successfully
 *       -1 if the saliency features are not computed successfully
 */
int compute_saliency_features(cv::Mat camera_image, int* centre_x, int* centre_y, bool debug_mode);





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
ControlClientPtr create_client(const std::string& topic_name);

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
void rotate_robot(double angle_degrees, ros::Publisher velocity_publisher, bool debug);

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
                        std::vector<std::vector<double>>& accelerations, std::vector<double>& durations);

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
int move_robot_head(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug);

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
 *       0 if the head is moved successfully
 *      -1 if the head is not moved successfully
 */
int move_robot_head_biological_motion(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, bool debug);

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
                        std::vector<double> positions);

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
                                        std::vector<std::vector<double>> accelerations);

/*  
 *   Function to move the robot head and wheels to a position specified by the head pitch and head yaw angles
 *   The function moves the head and wheels to the specified position using the control client
 *
 *   @param:
 *       head_client: the control client for the head
 *       joint_names: vector containing the joint names of the head
 *       duration: the duration of the movement
 *       positions: vector containing the joint angles of the position to move the head to
 *       rotate_robot: boolean to indicate if the robot should be rotated
 *       angle_radians: the angle in radians to rotate the robot
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */	
void move_robot_head_wheels_to_position(ControlClientPtr& head_client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions, bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);

/*  
 *   Function to move the robot head and wheels to a position specified by the head pitch and head yaw angles
 *   The function moves the head and wheels to the specified position using the minimum-jerk model of biological motion
 *
 *   @param:
 *       head_client: the control client for the head
 *       joint_names: vector containing the joint names of the head
 *       duration: the duration of the movement
 *       positions: vector containing the joint angles of the position to move the head to
 *       velocities: vector containing the joint velocities of the position to move the head to
 *       accelerations: vector containing the joint accelerations of the position to move the head to
 *       rotate_robot: boolean to indicate if the robot should be rotated
 *       angle_radians: the angle in radians to rotate the robot
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void move_robot_head_wheels_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, 
                                        double gesture_duration, std::vector<double> duration, 
                                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, 
                                        std::vector<std::vector<double>> accelerations,
                                        bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);

/* 
 *   Function to control the robot's head and wheels
 *   The function controls the robot's head and wheels to look at a specified point in the environment
 *
 *   @param:
 *       head_topic: the topic for the head
 *       head_pitch: the pitch angle of the head
 *       head_yaw: the yaw angle of the head
 *       gesture_duration: the duration of the gesture
 *       rotate_robot: boolean to indicate if the robot should be rotated
 *       angle_radians: the angle in radians to rotate the robot
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void control_robot_head_wheels(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, 
                            bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug);




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
int location_attention(float point_x, float point_y, float point_z, string topics_file, ros::Publisher velocity_publisher, bool debug);

/* 
 *   Function to execute the social attention
 *   The function moves the robot's head to look at the specified point in the environment centered around the detected faces.
 * 
 * @param:
 *    topics_file: the topics file
 *    realignment_threshold: the realignment threshold
 *    velocity_publisher: the velocity publisher
 *    social_control: the social control mode - saliency or random
 *    debug: the debug mode
 * 
 * @return:
 *   1 if the attention is executed successfully
 */
int social_attention(std::string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, int social_control, bool debug);

/* 
 *   Function to execute the scanning attention
 *   The function moves the robot's head to scan the environment.
 * 
 * @param:
 *    control_head_yaw: the yaw angle of the head
 *    control_head_pitch: the pitch angle of the head
 *    topics_file: the topics file
 *    velocity_publisher: the velocity publisher
 *    debug: the debug mode
 * 
 * @return:
 *   1 if the attention is executed successfully
 */
int scanning_attention(double control_head_yaw, double control_head_pitch, string topics_file, ros::Publisher velocity_publisher, bool debug);

/* 
 *   Function to execute the seeking attention
 *   The function moves the robot's head to seek the environment.
 * 
 * @param:
 *    topics_file: the topics file
 *    realignment_threshold: the realignment threshold
 *    velocity_publisher: the velocity publisher
 *    overt_attention_mode_pub: the overt attention mode publisher
 *    debug: the debug mode
 * 
 * @return:
 *   1 if the attention is executed successfully
 */
int seeking_attention(string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, ros::Publisher overt_attention_engagement_status_pub, bool debug);


/*  --------------------------------------------------
            UTILITIY CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to convert radians to degrees
 *   This function converts the angle in radians to degrees
 *
 * @param:
 *   radians: the angle in radians
 *
 * @return:
 *   the angle in degrees
 */
double degrees(double radians);

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
double radians(double degrees);
 
/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status);

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void prompt_and_continue();

/*  
 *   Function to handle the shutdown signal
 *   Cancels all active goals and shuts down ROS
 *  @param:
 *      sig: the signal
 *  @return:
 *      None
 */
void shut_down_handler(int sig);


#endif // OVERT_ATTENTION_INTERFACE_H