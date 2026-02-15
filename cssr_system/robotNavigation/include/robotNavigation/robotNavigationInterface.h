/* robotNavigationInterface.h - Robot navigation interface and imported libraries (Action-based)
 *
 * Author:  Birhanu Shimelis Girma, Carnegie Mellon University Africa
 * Email:   bgirmash@andrew.cmu.edu
 * Date:    February 05, 2026
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
   #include <fcntl.h>
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
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iomanip>

#include "geometry_msgs/Pose2D.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64.h>
#include <fstream>
#include <sstream>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// Include action headers for setGoal and setPose actions
#include <cssr_system/setGoalAction.h>
#include <cssr_system/setPoseAction.h>

using namespace std;
using namespace cv;
using namespace boost::algorithm;

/***************************************************************************************************************************

   ROS package name

****************************************************************************************************************************/
#define ROS_PACKAGE_NAME    "cssr_system"

// Software version
#define SOFTWARE_VERSION                    "v1.0"
/***************************************************************************************************************************

   General purpose definitions

****************************************************************************************************************************/
#define PI       3.14159

#define TRUE     1
#define FALSE    0

#define BFS_ALGORITHM               0
#define DIJKSTRA_ALGORITHM          1
#define ASTAR_ALGORITHM             2
#define DFS_ALGORITHM               3

#define BFS      0
#define DIJKSTRA 1
#define ASTAR    2
#define DFS      3

// Feedback publishing rate (Hz)
#define FEEDBACK_RATE 1.0


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

//#######################################################
// To add navigation mode related variables
extern std::string navigation_mode;

// SLAM mode publishers
extern ros::Publisher slam_goal_publisher;
extern ros::Publisher slam_initialpose_publisher;

// SLAM mode action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
extern MoveBaseClient* move_base_client;

//#####################################################


// Publisher for the velocity commands
extern ros::Publisher navigation_velocity_publisher;
extern ros::Publisher navigation_pelvis_publisher;

extern std::atomic<bool> is_moving;


#define PUBLISH_RATE 10

// Size of the map
extern int x_map_size;
extern int y_map_size;

extern int image_width;
extern int image_height;

extern double room_width;
extern double room_height;

// Window names to display the maps
extern string mapWindowName;

extern std::vector<double> leg_home_position;
extern std::vector<double> head_home_position;

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
#define GOING                 0
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
   double min_linear_velocity;
   double max_linear_velocity;
   double min_angular_velocity;
   double max_angular_velocity;
   double clearance;
   int   shortest_path_algorithm;
   bool  robot_available;
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

extern std::vector<int> robot_path;
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

extern std::vector<std::vector<int>> graph;




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


// Action-based smooth navigation function with preemption and feedback support
int executeSmoothWaypointNavigationWithFeedback(double start_x, double start_y, double start_theta,
                                                double goal_x, double goal_y, double goal_theta,
                                                ros::Publisher velocity_publisher, ros::Rate rate, bool debug,
                                                actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server);


/***************************************************************************************************************************

   Function declarations for mapping between the world and map coordinates

****************************************************************************************************************************/

void convertWorldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y, double room_width, double room_height, int image_width, int image_height);
void convertPixelToWorld(int pixel_x, int pixel_y, double& world_x, double& world_y, double room_width, double room_height, int image_width, int image_height) ;


/***************************************************************************************************************************

   Function declarations for mobile robot control

****************************************************************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry& msg);
void poseMessageReceived(const geometry_msgs::Pose2D &msg);
void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData);
void readObstacleData(char filename[], Mat &map);
void findMinimumVelocities(ros::Publisher pub, ros::Rate rate, double max_linear_velocity,  double max_angular_velocity);
void setOdometryPose(double x, double y, double z);
void goToPoseDQ     (double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate);
void goToPoseMIMO   (double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints);


/***************************************************************************************************************************

   Action server callback function declarations

****************************************************************************************************************************/

/*
 * Callback function for the setGoal action server
 * This function is called when a new goal is received on the /robotNavigation/set_goal action
 *
 * @param goal: The goal message containing the target position (goal_x, goal_y, goal_theta)
 * @param action_server: Pointer to the action server
 */
void setGoalActionCallback(const cssr_system::setGoalGoalConstPtr &goal,
                           actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server);

/*
 * Callback function for the setPose action server
 * This function is called when a new pose is received on the /robotNavigation/set_pose action
 *
 * @param goal: The goal message containing the pose (pose_x, pose_y, pose_theta)
 * @param action_server: Pointer to the action server
 */
void setPoseActionCallback(const cssr_system::setPoseGoalConstPtr &goal,
                           actionlib::SimpleActionServer<cssr_system::setPoseAction>* action_server);

/***************************************************************************************************************************
// SLAM mode utility functions

****************************************************************************************************************************/

void publishSlamGoal(double goal_x, double goal_y, double goal_theta);

void publishSlamInitialPose(double pose_x, double pose_y, double pose_theta);

int executeSlamNavigation(double goal_x, double goal_y, double goal_theta);

// SLAM navigation with action feedback support
int executeSlamNavigationWithFeedback(double goal_x, double goal_y, double goal_theta,
                                      actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server);

/***************************************************************************************************************************

   General purpose function declarations

****************************************************************************************************************************/

void displayErrorAndExit(char error_message[]);
void printMessageToFile(FILE *fp, char message[]);
int  signnum(double x);

#ifdef ROS
   int kbhit();
#endif

/*
 * Function to check if /robotLocalization/pose topic is available
 * @return: true if topic is available, false otherwise
 */
bool isPoseTopicAvailable();

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
int readConfigurationFile(string* environmentMapFile, string* configurationMapFile, int* pathPlanningAlgorithm, bool* socialDistanceMode, string* robot_topics, string* topics_filename, bool* debug_mode, string* robot_type, string* navigation_mode);

void printConfiguration(string environmentMapFile, string configurationMapFile, int pathPlanningAlgorithm, bool socialDistanceMode, string robot_topics, string topics_filename, bool debug_mode, string robot_type, string navigation_mode);

void saveWaypointMap(vector<int> compressionParams, Mat mapImageLarge, string fileName);

void markWaypointsOnMap(int pathPlanningAlgorithm, Mat mapImage, std::string output_filename, bool debug);

void markWaypointsOnConfigMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

void markWaypointsOnEnvironmentMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

int planRobotPath(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug);

void stabilizeWaist();

void stabilizeWaistContinuously();

int navigateToGoal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug);

// Action-based navigation function with feedback support
int navigateToGoalWithFeedback(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta,
                               int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug,
                               actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server);

ControlClientPtr createClient(const std::string& topic_name);

int goToHome(std::string actuator, std::string topics_filename, bool debug);

int moveRobot(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, ros::Publisher velocity_publisher, ros::Rate rate, bool debug);

// Action-based move robot function with feedback support
int moveRobotWithFeedback(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta,
                          ros::Publisher velocity_publisher, ros::Rate rate, bool debug,
                          actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server);

/*
 *   Function to round a floating point number to a specified number of decimal places
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
