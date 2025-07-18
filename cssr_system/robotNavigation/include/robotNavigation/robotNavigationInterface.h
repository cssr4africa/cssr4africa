/* robotNavigationInterface.h - robot navigation interfacing and imported libraries
 *
 * Author:  Birhanu Shimelis Girma, Carnegie Mellon University Africa
 * Email:   bgirmash@andrew.cmu.edu
 * Date:    June 05, 2025
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

#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H


#define ROS
 
#ifndef ROS
   #include <conio.h>
#else
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <sys/select.h>
   #include <termios.h>
   #include <sys/ioctl.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h> // for turtle1/teleport_absolute service
#include <turtlesim/SetPen.h>           // for turtle1/set_pen service
#include <std_srvs/Empty.h>             // for reset and clear services
#include <geometry_msgs/Twist.h>        // For geometry_msgs::Twist
#include <nav_msgs/Odometry.h>          // For nav_msgs::Odometry
#include <iomanip>                      // for std::setprecision and std::fixed

#include "cssr_system/setGoal.h"  // Include for the setGoal service
#include "geometry_msgs/Pose2D.h"       // For geometry_msgs::Pose2D
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64.h>  // Include for publishing Float64 messages
#include <fstream>
#include <sstream>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

#include <thread>
#include <atomic>

#include <queue>
#include <vector>
#include <climits>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <climits>
#include <string>
#include <opencv2/opencv.hpp>
#include <stack>

using namespace std;
using namespace cv;
using namespace boost::algorithm;

/***************************************************************************************************************************

   ROS package name

****************************************************************************************************************************/
#define ROS_PACKAGE_NAME    "cssr_system"              // Changed to cssr_system for integration

// Software version
#define SOFTWARE_VERSION                    "v1.0"
/***************************************************************************************************************************

   General purpose definitions 

****************************************************************************************************************************/
#define PI       3.14159

#define TRUE     1
#define FALSE    0

#define BFS_ALGORITHM               0         // Breadth First Search Algorithm
#define DIJKSTRA_ALGORITHM          1        // Dijkstra's Algorithm
#define ASTAR_ALGORITHM             2       // A* Algorithm
#define DFS_ALGORITHM               3

#define BFS      0
#define DIJKSTRA 1
#define ASTAR    2
#define DFS      3


extern std::string nodeName;   
// Directory where the package is located
extern std::string packagedir;

// Stores the robot location in the environment (x, y, theta)
extern std::vector<double> robotPose;

// Configuration parameters
extern std::string implementation_platform;
extern std::string environmentMapFile;
extern std::string configurationMapFile;
extern int pathPlanningAlgorithm;
extern bool socialDistanceMode;
extern std::string robot_topics;
extern string topics_filename;
extern bool verbose_mode;
extern std::string robot_type;


// Publisher for the velocity commands
extern ros::Publisher navigation_velocity_publisher;
extern ros::Publisher navigation_pelvis_publisher;

extern std::atomic<bool> is_moving;  // Flag to indicate if the robot is moving


#define PUBLISH_RATE 10

// Size of the map
extern int x_map_size;
extern int y_map_size;

extern int image_width;
extern int image_height;

extern double room_width;  // Width of the room in meters
extern double room_height;  // Height of the room in meters

// Window names to display the maps
extern string mapWindowName; 

extern std::vector<double> leg_home_position;  // Hip pitch, hip roll, knee pitch
extern std::vector<double> head_home_position;   // Head pitch and yaw
// Mat images to display the maps
// extern Mat mapImage;
// extern Mat mapImageColor;
// extern Mat mapImageLarge;
// extern Mat configurationSpaceImage;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;

extern double                  current_x;
extern double                  current_y;
extern double                  current_theta;

extern double x, y, theta;
extern double x_start, y_start, theta_start;
extern double x_goal, y_goal, theta_goal;
extern Mat mapImage;
extern Mat configurationSpaceImage;


/***************************************************************************************************************************

   Definitions for reading locomotion parameter data 

****************************************************************************************************************************/

#define MAX_FILENAME_LENGTH 200
#define STRING_LENGTH       200
#define KEY_LENGTH           40
#define NUMBER_OF_KEYS       15
#define GOING                 0  // used to switch between angle_tolerance_going and angle_tolerance_orienting
#define ORIENTING             1

typedef char keyword[KEY_LENGTH];

typedef struct {
   double position_tolerance;
   double position_tolerance_goal;
   double angle_tolerance_orienting;
   double angle_tolerance_going;
   double position_gain_dq; 
   double angle_gain_dq; 
   double position_gain_mimo; 
   double angle_gain_mimo;
   double min_linear_velocity;                           // m/s       ... from calibration; less than this and the motors are not actuated
   double max_linear_velocity;                           // m/s       ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
   double min_angular_velocity;                          // radians/s ... from calibration; less than this and the motors are not actuated
   double max_angular_velocity;                          // radians/s ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
   double clearance;                                     // m         ... required clearance between the robot and an obstacle
   int   shortest_path_algorithm;                       // BFS, Dijkstra, ASTAR
   bool  robot_available;                               // true/false... determine if a physical robot is available
} locomotionParameterDataType;

/***************************************************************************************************************************

   Definitions for paths and waypoints 

****************************************************************************************************************************/

#define CURVATURE_THRESHOLD       0.25
#define SUPPRESSION_FACTOR        30

#define CROSS_HAIR_SIZE 10

typedef struct  {
   int x;
   int y;
} pointType;

typedef struct  {
   int x;
   int y;
   double theta;
} waypointType;

extern std::vector<int> robot_path; // Path to store the path
extern std::vector<pointType> valid_path;
extern std::vector<waypointType> valid_waypoints;

/***************************************************************************************************************************

   Definitions for the graph data structure

****************************************************************************************************************************/

#define MANHATTAN_HEURISTIC 1
#define EUCLIDEAN_HEURISTIC 2

// 4-way movement (BFS)
extern int directions_4_way[4][2];

// 8-way movement (Dijkstra, A*)
extern int directions_8_way[8][2];

extern std::vector<std::vector<int>> graph;  // Graph to store the map





extern locomotionParameterDataType locomotionParameterData;

extern geometry_msgs::Twist msg;

/***************************************************************************************************************************

   Function declarations for the graph data structure 

****************************************************************************************************************************/

void buildGraphFromMap(cv::Mat& img, std::vector<std::vector<int>>& graph, int algorithm);
int manhattanDistanceHeuristic(int start, int node, int goal, int cols);
int euclideanDistanceHeuristic(int start, int node, int goal, int cols);
void drawPathOnMap(const std::vector<waypointType>& valid_waypoints, const cv::Mat& img, const std::string& output_path);
double pathOrientation(std::vector<pointType>& valid_path, int i);
void computeWaypoints(std::vector<int>& robot_path, std::vector<pointType>& valid_path, std::vector<waypointType>& valid_waypoints, int suppression_factor, double curvature_angle);
void printWaypoints(std::vector<waypointType>& valid_waypoints, double room_width, double room_height, int image_width, int image_height);
std::vector<int> astar(int start, int goal, const std::vector<std::vector<int>>& graph, int cols, int heuristic_type);
std::vector<int> bfs(int start, int goal, const std::vector<std::vector<int>>& graph);
std::vector<int> dijkstra(int start, int goal, const std::vector<std::vector<int>>& graph);
std::vector<int> dfs(int start, int goal, const std::vector<std::vector<int>>& graph);


/***************************************************************************************************************************/

// Helper functions for improved waypoint generation
double calculateOptimalApproachAngle(const std::vector<pointType>& path, int current_idx, int lookahead);
std::vector<waypointType> optimizeWaypointSequence(const std::vector<waypointType>& candidates, double min_turning_radius);

// New smooth navigation function
int executeSmoothWaypointNavigation(double start_x, double start_y, double start_theta, 
                                   double goal_x, double goal_y, double goal_theta,
                                   ros::Publisher velocity_publisher, ros::Rate rate, bool debug);



                                   
/***************************************************************************************************************************

   Function declarations for mapping between the world and map coordinates

****************************************************************************************************************************/

void convertWorldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y, double room_width, double room_height, int image_width, int image_height);
void convertPixelToWorld(int pixel_x, int pixel_y, double& world_x, double& world_y, double room_width, double room_height, int image_width, int image_height) ;


/***************************************************************************************************************************

   Function declarations for mobile robot control 

****************************************************************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry& msg); // callback: executed each time a new message arrives on the odom topic
void poseMessageReceived(const geometry_msgs::Pose2D &msg);
void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData);
void readObstacleData(char filename[], Mat &map);
void findMinimumVelocities(ros::Publisher pub, ros::Rate rate, double max_linear_velocity,  double max_angular_velocity);
void setOdometryPose(double x, double y, double z);
void goToPoseDQ     (double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate);
void goToPoseMIMO   (double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints);


/***************************************************************************************************************************

   Service callback function declaration

****************************************************************************************************************************/

bool setGoal(cssr_system::setGoal::Request  &service_request, cssr_system::setGoal::Response &service_response);

/***************************************************************************************************************************

   General purpose function declarations 

****************************************************************************************************************************/

void displayErrorAndExit(char error_message[]);
void printMessageToFile(FILE *fp, char message[]);
int  signnum(double x); // return the sign of a number

#ifdef ROS
   int kbhit();
#endif



/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void readRobotPoseInput(std::vector<double>& robot_pose_input);

void writeRobotPoseInput(std::vector<double>& robot_pose_input);

/*  
 *   Function to extract the topic from the topics file
 *   The function reads the topics file and extracts the topic for the specified key.
 *
 *   @param:
 *       key: the key to search for in the topics file
 *       topic_file_name: the topics filename
 *       topic_name: the topic name extracted
 *
 *   @return:
 *       0 if successful, 1 otherwise
 */
int extractTopic(string key, string topic_file_name, string *topic_name);

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions);
int readConfigurationFile(string* environmentMapFile, string* configurationMapFile, int* pathPlanningAlgorithm, bool* socialDistanceMode, string* robot_topics, string* topics_filename, bool* debug_mode, string* robot_type);

void printConfiguration(string environmentMapFile, string configurationMapFile, int pathPlanningAlgorithm, bool socialDistanceMode, string robot_topics, string topics_filename, bool debug_mode, string robot_type);

void saveWaypointMap(vector<int> compressionParams, Mat mapImageLarge, string fileName);

void markWaypointsOnMap(int pathPlanningAlgorithm, Mat mapImage, std::string output_filename, bool debug);

void markWaypointsOnConfigMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

void markWaypointsOnEnvironmentMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

int planRobotPath(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

void stabilizeWaist();

void stabilizeWaistContinuously();

int navigateToGoal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug);

ControlClientPtr createClient(const std::string& topic_name);

int goToHome(std::string actuator, std::string topics_filename, bool debug);

int moveRobot(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, ros::Publisher velocity_publisher, ros::Rate rate, bool debug);

/*
 *   Function to round a doubleing point number to a specified number of decimal places
 *
 *  @param:
 *     value: the value to be rounded
 *     decimal_places: the number of decimal places
 *  @return:
 *     the rounded value
 * 
 */
double roundFloatingPoint(double value, int decimal_places);

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
void promptAndExit(int status);

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void promptAndContinue();

void moveToPosition(ControlClientPtr& head_client, const std::vector<std::string>& head_joint_names, std::vector<double> head_positions, 
                        ControlClientPtr& left_arm_client, const std::vector<std::string>& left_arm_joint_names, std::vector<double> left_arm_positions, 
                        ControlClientPtr& right_arm_client, const std::vector<std::string>& right_arm_joint_names, std::vector<double> right_arm_positions, 
                        ControlClientPtr& leg_client, const std::vector<std::string>& leg_joint_names, std::vector<double> leg_positions, 
                        double duration);

void moveOneActuatorToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions);


ControlClientPtr createClient(const std::string& topic_name) ;

void moveRobotActuatorsToDefault();
#endif // ROBOT_NAVIGATION_H
