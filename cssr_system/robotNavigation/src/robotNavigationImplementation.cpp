/* robotNavigationImplementation.cpp - Robot navigation functions implementation (Action-based).
 *
 * Author:   Birhanu Shimelis Girma, Carnegie Mellon University Africa
 * Email:    bgirmash@andrew.cmu.edu
 * Date:     February 05, 2026
 * Version:  v1.0
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


#include "robotNavigation/robotNavigationInterface.h"

// Directory where the package is located
std::string packagedir;

// Coordinates of the robot in the world (x, y, z, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robotPose = {2.0, 7.8, 270.0};

// Flag to indicate if a waypoint is being followed
bool waypointFlag;

// Configuration parameters
std::string implementation_platform;
std::string environmentMapFile;
std::string configurationMapFile;
int pathPlanningAlgorithm;
bool socialDistanceMode;
std::string robot_topics;
string topics_filename;
bool verbose_mode;

// Global variable to track last pose update time
ros::Time last_pose_update_time;

// Global variables moved from application file
double x;
double y;
double theta;

int suppression_factor = 5;
// Robot pose input which contains the x, y, and theta values of the start location
double x_start;
double y_start;
double theta_start;

// Service request which contains the x, y, and theta values of the goal location
double x_goal;
double y_goal;
double theta_goal;
std::string robot_type;
std::string nodeName = "robotNavigation";
Mat mapImage;
Mat configurationSpaceImage;

// ########################################################
// Navigation mode variables
std::string navigation_mode;

// SLAM mode publishers
ros::Publisher slam_goal_publisher;
ros::Publisher slam_initialpose_publisher;

// SLAM mode action client
MoveBaseClient* move_base_client = nullptr;
// ########################################################

// Publisher for the velocity commands
ros::Publisher navigation_velocity_publisher;
ros::Publisher navigation_pelvis_publisher;

std::atomic<bool> is_moving(false);

// Size of the map
int x_map_size;
int y_map_size;

int image_width;
int image_height;

double room_width;
double room_height;

// Window names to display the maps
string mapWindowName = "Environment Map";


locomotionParameterDataType locomotionParameterData;

geometry_msgs::Twist msg;

std::vector<double> leg_home_position = {0.0, 0.0, 0.0};
std::vector<double> head_home_position = {0.0, 0.0};

std::vector<std::vector<int>> graph;
std::vector<int> robot_path;
std::vector<pointType> valid_path;
std::vector<waypointType> valid_waypoints;

// 4-way movement (BFS)
int directions_4_way[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// 8-way movement (Dijkstra, A*)
int directions_8_way[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

/*
 * Function to check if /robotLocalization/pose topic is available
 * @return: true if topic is available, false otherwise
 */
bool isPoseTopicAvailable() {
    ros::master::V_TopicInfo master_topics;
    if (!ros::master::getTopics(master_topics)) {
        return false;
    }

    for (const auto& topic : master_topics) {
        if (topic.name == "/robotLocalization/pose") {
            return true;
        }
    }

    return false;
}

/***************************************************************************************************************************

   Action callback function for setGoal action

   This function is called when a new goal is received on the /robotNavigation/set_goal action.
   It handles both SLAM and CAD navigation modes with feedback support.

****************************************************************************************************************************/

void setGoalActionCallback(const cssr_system::setGoalGoalConstPtr &goal,
                           actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server) {
    // Extract goal parameters
    x_goal = goal->goal_x;
    y_goal = goal->goal_y;
    theta_goal = goal->goal_theta;
    theta_goal = radians(theta_goal);

    x_goal = roundFloatingPoint(x_goal, 1);
    y_goal = roundFloatingPoint(y_goal, 1);

    ROS_INFO("Navigation mode: %s", navigation_mode.c_str());

    // Initialize result
    cssr_system::setGoalResult result;
    result.navigation_goal_success = 0;
    result.final_x = current_x;
    result.final_y = current_y;
    result.final_theta = degrees(current_theta);

    // SLAM mode - use move_base action client with feedback
    if (navigation_mode == "SLAM") {
        ROS_INFO("Using SLAM navigation mode");

        if (move_base_client == nullptr || !move_base_client->isServerConnected()) {
            ROS_ERROR("move_base action server is not available. Cannot navigate.");
            result.navigation_goal_success = 0;
            action_server->setAborted(result, "move_base action server not available");
            return;
        }

        int navigation_result = executeSlamNavigationWithFeedback(x_goal, y_goal, theta_goal, action_server);

        result.navigation_goal_success = navigation_result;
        result.final_x = current_x;
        result.final_y = current_y;
        result.final_theta = degrees(current_theta);

        if (navigation_result == 1) {
            action_server->setSucceeded(result, "Goal reached successfully in SLAM mode");
        } else {
            action_server->setAborted(result, "Failed to reach goal in SLAM mode");
        }
        return;
    }

    // CAD mode - use existing logic with action feedback
    ROS_INFO("Using CAD navigation mode");

    // Check if goal is same as current position (within tolerance)
    double distance_to_goal = sqrt(pow(x_goal - current_x, 2) + pow(y_goal - current_y, 2));
    double angle_difference = fabs(theta_goal - current_theta);

    while (angle_difference > M_PI) angle_difference -= 2 * M_PI;
    while (angle_difference < -M_PI) angle_difference += 2 * M_PI;
    angle_difference = fabs(angle_difference);

    if (distance_to_goal <= locomotionParameterData.position_tolerance_goal &&
        angle_difference <= locomotionParameterData.angle_tolerance_orienting) {

        ROS_INFO("Robot is already at the goal location within tolerance. Distance: %.3f m, Angle diff: %.3f degrees",
                distance_to_goal, angle_difference);

        robotPose[0] = current_x;
        robotPose[1] = current_y;
        robotPose[2] = degrees(current_theta);
        writeRobotPoseInput(robotPose);

        result.navigation_goal_success = 1;
        result.final_x = current_x;
        result.final_y = current_y;
        result.final_theta = degrees(current_theta);
        action_server->setSucceeded(result, "Already at goal location");
        return;
    }

    // Check if pose topic is still available
    if (!isPoseTopicAvailable()) {
        ROS_ERROR("Navigation request rejected: /robotLocalization/pose topic is not available");
        result.navigation_goal_success = 0;
        action_server->setAborted(result, "Pose topic not available");
        return;
    }

    // Check if we're receiving recent pose updates
    ros::Time current_time = ros::Time::now();
    if (!last_pose_update_time.isZero()) {
        double time_since_last_pose = (current_time - last_pose_update_time).toSec();
        if (time_since_last_pose > 3.0) {
            ROS_ERROR("Navigation request rejected: No pose updates received in %.1f seconds", time_since_last_pose);
            result.navigation_goal_success = 0;
            action_server->setAborted(result, "No recent pose updates");
            return;
        }
    }

    x_start = current_x;
    x_start = roundFloatingPoint(x_start, 1);
    y_start = current_y;
    y_start = roundFloatingPoint(y_start, 1);
    theta_start = current_theta;

    if(x_start > (double) x_map_size/100 || y_start > (double) y_map_size/100){
        ROS_ERROR("Robot pose is outside the map");
        result.navigation_goal_success = 0;
        action_server->setAborted(result, "Robot pose outside map");
        return;
    }
    if(x_goal > (double) x_map_size/100 || y_goal > (double) y_map_size/100){
        ROS_ERROR("Goal location is outside the map");
        result.navigation_goal_success = 0;
        action_server->setAborted(result, "Goal location outside map");
        return;
    }

    int navigation_goal_success = navigateToGoalWithFeedback(x_start, y_start, theta_start, x_goal, y_goal, theta_goal,
                                                            pathPlanningAlgorithm, mapImage, configurationSpaceImage,
                                                            navigation_velocity_publisher, verbose_mode, action_server);

    if(navigation_goal_success == 1){
        robotPose[0] = current_x;
        robotPose[1] = current_y;
        robotPose[2] = degrees(current_theta);
        writeRobotPoseInput(robotPose);
    }

    result.navigation_goal_success = navigation_goal_success;
    result.final_x = current_x;
    result.final_y = current_y;
    result.final_theta = degrees(current_theta);

    if (navigation_goal_success == 1) {
        action_server->setSucceeded(result, "Goal reached successfully");
    } else if (action_server->isPreemptRequested()) {
        action_server->setPreempted(result, "Navigation preempted");
    } else {
        action_server->setAborted(result, "Navigation failed");
    }

    ROS_INFO("Response from /robotNavigation/set_goal action: [%d]\n", navigation_goal_success);
}

/***************************************************************************************************************************

   Action callback function for setPose action

   This function is called when a new pose is received on the /robotNavigation/set_pose action.

****************************************************************************************************************************/

void setPoseActionCallback(const cssr_system::setPoseGoalConstPtr &goal,
                           actionlib::SimpleActionServer<cssr_system::setPoseAction>* action_server) {
    double pose_x = goal->pose_x;
    double pose_y = goal->pose_y;
    double pose_theta = goal->pose_theta;

    cssr_system::setPoseResult result;
    cssr_system::setPoseFeedback feedback;

    feedback.status = "Setting robot pose...";
    action_server->publishFeedback(feedback);

    ROS_INFO("Setting robot pose to: x=%.2f, y=%.2f, theta=%.2f", pose_x, pose_y, pose_theta);

    if (navigation_mode == "SLAM") {
        feedback.status = "Publishing to SLAM initialpose topic...";
        action_server->publishFeedback(feedback);

        publishSlamInitialPose(pose_x, pose_y, pose_theta);
        ROS_INFO("Pose forwarded to ROS navigation stack initialpose topic");

        feedback.status = "Pose set successfully in SLAM mode";
        action_server->publishFeedback(feedback);
    } else {
        feedback.status = "CAD mode - pose setting handled by robotLocalization";
        action_server->publishFeedback(feedback);
        ROS_INFO("CAD mode - pose setting handled by robotLocalization");
    }

    result.pose_set_success = 1;
    action_server->setSucceeded(result, "Pose set successfully");
}

/*
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void readRobotPoseInput(std::vector<double>& robot_pose_input){
    bool debug_mode = false;

    std::string data_file = "robotPose.dat";
    std::string data_path;
    std::string data_path_and_file;
    std::string x_key = "x";
    std::string y_key = "y";
    std::string theta_key = "theta";
    std::string x_value;
    std::string y_value;
    std::string z_value;
    std::string theta_value;

    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("Unable to open the data file %s\n", data_path_and_file.c_str());
        promptAndExit(1);
    }

    std::string data_line_read;
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        if (param_key == x_key){
            x_value = param_value;
            robot_pose_input[0] = std::stod(param_value);
        }
        else if (param_key == y_key){
            y_value = param_value;
            robot_pose_input[1] = std::stod(param_value);
        }
        else if (param_key == theta_key){
            theta_value = param_value;
            robot_pose_input[2] = std::stod(param_value);
        }
    }
    data_if.close();

    if (debug_mode){
        printf("Robot location input:\n");
        printf("\tX: %.2f\n", robot_pose_input[0]);
        printf("\tY: %.2f\n", robot_pose_input[1]);
        printf("\tTheta: %.2f\n", robot_pose_input[2]);
    }
}

/*
 *   Function to write the robot pose from to a file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void writeRobotPoseInput(std::vector<double>& robot_pose_input){
    bool debug_mode = false;

    std::string data_file = "robotPose.dat";
    std::string data_path;
    std::string data_path_and_file;

    std::string x_key = "x";
    std::string y_key = "y";
    std::string theta_key = "theta";

    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    std::ofstream data_of(data_path_and_file.c_str());
    if (!data_of.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        promptAndExit(1);
    }

    data_of << x_key << "\t\t\t" << robot_pose_input[0] << std::endl;
    data_of << y_key << "\t\t\t" << robot_pose_input[1] << std::endl;
    data_of << theta_key << "\t\t" << robot_pose_input[2] << std::endl;

    data_of.close();
}

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
int extractTopic(string key, string topic_file_name, string *topic_name){
    bool debug = false;

    std::string topic_path;
    std::string topic_path_and_file;
    std::string topic_value = "";

    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    topic_path += "/robotNavigation/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) ROS_INFO("Topic file is %s\n", topic_path_and_file.c_str());

    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;
    while(std::getline(topic_if, topic_line_read)){
        std::istringstream iss(topic_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        if (param_key == key) {
            topic_value = param_value;
            break;
        }
    }
    topic_if.close();

    if (topic_value == ""){
        ROS_ERROR("Unable to find a valid topic for actuator: %s. Please check the topics file.\n", key.c_str());
        return 1;
    }

    *topic_name = topic_value;
    return 0;
}

/*
 *   Function to move an actuator to a position when using linear interpolation
 */
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration,
                        bool open_hand, string hand, string hand_topic,
                        const std::string& position_name, std::vector<double> positions){
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    ControlClientPtr hand_client;

    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration));

    return;
}


int readConfigurationFile(string* environmentMapFile, string* configurationMapFile, int* path_planning_algorithm, bool* socialDistanceMode, string* robot_topics, string* topics_filename, bool* debug_mode, string* robot_type, string* navigationMode) {
    std::string config_file = "robotNavigationConfiguration.ini";
    std::string config_path;
    std::string config_path_and_file;

    std::string environment_map_file_key = "environmentMap";
    std::string configuration_map_file_key = "configurationMap";
    std::string path_planning_algorithm_key = "pathPlanning";
    std::string social_distance_mode_key = "socialDistance";
    std::string robot_topics_key = "robotTopics";
    std::string verbose_mode_key = "verboseMode";
    std::string robot_type_key = "robotType";
    std::string navigation_mode_key = "navigationMode";

    std::string environment_map_file_value;
    std::string configuration_map_file_value;
    std::string path_planning_algorithm_value;
    std::string social_distance_mode_value;
    std::string robot_topics_value;
    std::string verbose_mode_value;
    std::string robot_type_value;
    std::string navigation_mode_value;

    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
    #endif

    config_path += "/robotNavigation/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;
    *topics_filename = *robot_topics;

    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        return 1;
    }

    std::string data_line_read;
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key, param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        if (param_key == environment_map_file_key){
            environment_map_file_value = param_value;
            *environmentMapFile = param_value;
        }
        else if (param_key == configuration_map_file_key){
            configuration_map_file_value = param_value;
            *configurationMapFile = param_value;
        }
        else if (param_key == path_planning_algorithm_key){
            path_planning_algorithm_value = param_value;
            boost::algorithm::to_lower(param_value);
            if(param_value == "bfs"){
                *path_planning_algorithm = BFS_ALGORITHM;
            }
            else if((param_value == "dijkstra" || param_value == "dijsktra")){
                *path_planning_algorithm = DIJKSTRA_ALGORITHM;
            }
            else if((param_value == "a*") || (param_value == "a-star") || (param_value == "a_star") || (param_value == "astar")){
                *path_planning_algorithm = ASTAR_ALGORITHM;
            }
            else if(param_value == "dfs"){
                *path_planning_algorithm = DFS_ALGORITHM;
            }
            else{
                printf("Path planning algorithm value not supported. Supported values are: bfs, dijkstra, dfs, and a*\n");
                return 1;
            }
        }
        else if (param_key == social_distance_mode_key){
            boost::algorithm::to_lower(param_value);
            social_distance_mode_value = param_value;
            if(social_distance_mode_value == "true"){
                *socialDistanceMode = true;
            }
            else if(social_distance_mode_value == "false"){
                *socialDistanceMode = false;
            }
            else{
                printf("Social distance mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }
        else if (param_key == robot_topics_key){
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }
        else if (param_key == verbose_mode_key){
            boost::algorithm::to_lower(param_value);
            verbose_mode_value = param_value;
            if(verbose_mode_value == "true"){
                *debug_mode = true;
            }
            else if(verbose_mode_value == "false"){
                *debug_mode = false;
            }
            else{
                printf("Verbose mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }
        else if (param_key == robot_type_key) {
            robot_type_value = param_value;
            *robot_type = param_value;
        }
        else if (param_key == navigation_mode_key) {
            navigation_mode_value = param_value;
            boost::algorithm::to_lower(param_value);
            if(param_value == "cad") {
                *navigationMode = "CAD";
            }
            else if(param_value == "slam") {
                *navigationMode = "SLAM";
            }
            else {
                printf("Navigation mode value not supported. Supported values are: CAD and SLAM. Using default CAD.\n");
                *navigationMode = "CAD";
            }
        }
    }
    data_if.close();

    if(*environmentMapFile == "" || *configurationMapFile == "" || *robot_topics == ""){
        printf("Unable to find a valid configuration. Verify you have values in the configuration.\n");
        return 1;
    }

    *topics_filename = *robot_topics;

    if(*robot_type != "new" && *robot_type != "old") {
        ROS_WARN("Invalid robotType value: %s. Using default 'old'", robot_type_value.c_str());
        *robot_type = "old";
    }

    return 0;
}

void printConfiguration(string environmentMapFile, string configurationMapFile, int pathPlanningAlgorithm, bool socialDistanceMode, string robot_topics, string topics_filename, bool debug_mode, string robot_type, string navigation_mode){
    extern std::string nodeName;
    ROS_INFO("%s: Environment Map File: %s", nodeName.c_str(), environmentMapFile.c_str());
    ROS_INFO("%s: Configuration Map File: %s", nodeName.c_str(), configurationMapFile.c_str());
    ROS_INFO("%s: Path Planning Algorithm: %s", nodeName.c_str(),
             pathPlanningAlgorithm == BFS_ALGORITHM ? "BFS" :
             pathPlanningAlgorithm == DIJKSTRA_ALGORITHM ? "Dijkstra" :
             pathPlanningAlgorithm == ASTAR_ALGORITHM ? "A*" :
             pathPlanningAlgorithm == DFS_ALGORITHM ? "DFS" : "Unknown");
    ROS_INFO("%s: Social Distance Mode: %s", nodeName.c_str(), socialDistanceMode ? "true" : "false");
    ROS_INFO("%s: Robot Topics: %s", nodeName.c_str(), robot_topics.c_str());
    ROS_INFO("%s: Topics Filename: %s", nodeName.c_str(), topics_filename.c_str());
    ROS_INFO("%s: Verbose Mode: %s", nodeName.c_str(), debug_mode ? "true" : "false");
    ROS_INFO("%s: Robot Type: %s", nodeName.c_str(), robot_type.c_str());
    ROS_INFO("%s: Navigation Mode: %s", nodeName.c_str(), navigation_mode.c_str());
}

/******************************************************************************

global variables with the current robot pose

*******************************************************************************/

double                  initial_x        = 0;
double                  initial_y        = 0;
double                  initial_theta    = 0;
double                  current_x        = 0;
double                  current_y        = 0;
double                  current_theta    = 0;
double                  odom_x           = 0;
double                  odom_y           = 0;
double                  odom_theta       = 0;
double                  adjustment_x     = 0;
double                  adjustment_y     = 0;
double                  adjustment_theta = 0;

/******************************************************************************

odomMessageReceived

Callback function, executed each time a new pose message arrives

*******************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry &msg)
{
   bool debug = true;

   double x, y;

   odom_x = msg.pose.pose.position.x;
   odom_y = msg.pose.pose.position.y;
   odom_theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

   x = odom_x + adjustment_x - initial_x;
   y = odom_y + adjustment_y - initial_y;

   current_x = x * cos(adjustment_theta) + y * -sin(adjustment_theta);
   current_y = x * sin(adjustment_theta) + y * cos(adjustment_theta);

   current_x += initial_x;
   current_y += initial_y;

   current_theta = odom_theta + adjustment_theta;

   if (current_theta < -PI)
      current_theta += 2 * PI;
   else if (current_theta > PI)
      current_theta -= 2 * PI;

   if (debug)
   {
      ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f", current_x, current_y, current_theta);
   }
}

void poseMessageReceived(const geometry_msgs::Pose2D &msg)
{
    bool debug = true;

    current_x = msg.x;
    current_y = msg.y;
    current_theta = radians(msg.theta);

    last_pose_update_time = ros::Time::now();
}


/*******************************************************************************

readLocomotionParameterData

Read locomotion parameters from file

*******************************************************************************/

void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData)
{
   bool debug = true;
   int i;
   int j;
   int k;

   keyword keylist[NUMBER_OF_KEYS] = {
       "position_tolerance",
       "position_tolerance_goal",
       "angle_tolerance_orienting",
       "angle_tolerance_going",
       "position_gain_dq",
       "angle_gain_dq",
       "position_gain_mimo",
       "angle_gain_mimo",
       "min_linear_velocity",
       "max_linear_velocity",
       "min_angular_velocity",
       "max_angular_velocity",
       "clearance",
       "shortest_path_algorithm",
       "robot_available"};

   keyword key;
   keyword value;
   keyword robot_state;

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
     printf("Error can't open locomotion parameter file %s\n", filename);
     promptAndExit(0);
   }

   locomotionParameterData->position_tolerance = 0.025;
   locomotionParameterData->position_tolerance_goal = 0.01;
   locomotionParameterData->angle_tolerance_orienting = 0.075;
   locomotionParameterData->angle_tolerance_going = 0.075;
   locomotionParameterData->position_gain_dq = 0.3;
   locomotionParameterData->angle_gain_dq = 0.3;
   locomotionParameterData->position_gain_mimo = 0.2;
   locomotionParameterData->angle_gain_mimo = 0.5;
   locomotionParameterData->min_linear_velocity = 0.015;
   locomotionParameterData->max_linear_velocity = 0.5;
   locomotionParameterData->min_angular_velocity = 0.09;
   locomotionParameterData->max_angular_velocity = 1.0;
   locomotionParameterData->clearance = 0.05;
   locomotionParameterData->shortest_path_algorithm = ASTAR;
   locomotionParameterData->robot_available = FALSE;

   for (i = 0; i < NUMBER_OF_KEYS; i++)
   {
        if (fgets(input_string, STRING_LENGTH, fp_config) == NULL)
        {
            printf("Error: Unable to read all configuration parameters. Expected %d parameters, but could only read %d\n",
                NUMBER_OF_KEYS, i);
            fclose(fp_config);
            return;
        }

     sscanf(input_string, " %s", key);

     for (j = 0; j < (int)strlen(key); j++)
        key[j] = tolower(key[j]);

     for (j = 0; j < NUMBER_OF_KEYS; j++)
     {
        if (strcmp(key, keylist[j]) == 0)
        {
           switch (j)
           {
           case 0:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance));
              break;
           case 1:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance_goal));
              break;
           case 2:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_orienting));
              break;
           case 3:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_going));
              break;
           case 4:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_dq));
              break;
           case 5:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_dq));
              break;
           case 6:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_mimo));
              break;
           case 7:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_mimo));
              break;
           case 8:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_linear_velocity));
              break;
           case 9:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_linear_velocity));
              break;
           case 10:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_angular_velocity));
              break;
           case 11:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_angular_velocity));
              break;
           case 12:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->clearance));
              break;
           case 13:
              sscanf(input_string, " %s %s", key, value);
              for (k = 0; k < (int)strlen(value); k++)
                 value[k] = tolower(value[k]);
              if (strcmp(value, "bfs") == 0)
                 locomotionParameterData->shortest_path_algorithm = BFS;
              else if (strcmp(value, "dijkstra") == 0)
                 locomotionParameterData->shortest_path_algorithm = DIJKSTRA;
              else if (strcmp(value, "astar") == 0)
                 locomotionParameterData->shortest_path_algorithm = ASTAR;
              else if (strcmp(value, "dfs") == 0)
                 locomotionParameterData->shortest_path_algorithm = DFS;
              break;
           case 14:
              sscanf(input_string, " %s %s", key, robot_state);
              for (k = 0; k < (int)strlen(robot_state); k++)
                 robot_state[k] = tolower(robot_state[k]);
              if (strcmp(robot_state, "true") == 0)
                 locomotionParameterData->robot_available = TRUE;
              else
                 locomotionParameterData->robot_available = FALSE;
              break;
           }
        }
     }
   }

   if (debug)
   {
   ROS_INFO("%s: POSITION_TOLERANCE:        %.2f", nodeName.c_str(), locomotionParameterData->position_tolerance);
   ROS_INFO("%s: POSITION_TOLERANCE_GOAL:   %.2f", nodeName.c_str(), locomotionParameterData->position_tolerance_goal);
   ROS_INFO("%s: ANGLE_TOLERANCE_ORIENTING: %.2f", nodeName.c_str(), locomotionParameterData->angle_tolerance_orienting);
   ROS_INFO("%s: ANGLE_TOLERANCE_GOING:     %.2f", nodeName.c_str(), locomotionParameterData->angle_tolerance_going);
   ROS_INFO("%s: POSITION_GAIN_DQ:          %.2f", nodeName.c_str(), locomotionParameterData->position_gain_dq);
   ROS_INFO("%s: ANGLE_GAIN_DQ:             %.2f", nodeName.c_str(), locomotionParameterData->angle_gain_dq);
   ROS_INFO("%s: POSITION_GAIN_MIMO:        %.2f", nodeName.c_str(), locomotionParameterData->position_gain_mimo);
   ROS_INFO("%s: ANGLE_GAIN_MIMO:           %.2f", nodeName.c_str(), locomotionParameterData->angle_gain_mimo);
   ROS_INFO("%s: MIN_LINEAR_VELOCITY:       %.2f", nodeName.c_str(), locomotionParameterData->min_linear_velocity);
   ROS_INFO("%s: MAX_LINEAR_VELOCITY:       %.2f", nodeName.c_str(), locomotionParameterData->max_linear_velocity);
   ROS_INFO("%s: MIN_ANGULAR_VELOCITY:      %.2f", nodeName.c_str(), locomotionParameterData->min_angular_velocity);
   ROS_INFO("%s: MAX_ANGULAR_VELOCITY:      %.2f", nodeName.c_str(), locomotionParameterData->max_angular_velocity);
   ROS_INFO("%s: CLEARANCE:                 %.2f", nodeName.c_str(), locomotionParameterData->clearance);
   ROS_INFO("%s: SHORTEST_PATH_ALGORITHM    %d", nodeName.c_str(), locomotionParameterData->shortest_path_algorithm);
   ROS_INFO("%s: ROBOT_AVAILABLE            %d", nodeName.c_str(), locomotionParameterData->robot_available);
    }
}

/***************************************************************************************************************************

   Definitions for paths and waypoints

****************************************************************************************************************************/


void buildGraphFromMap(cv::Mat& img, std::vector<std::vector<int>>& graph, int algorithm) {
    int rows = img.rows;
    int cols = img.cols;

    int (*directions)[2];
    int num_directions;

    if (algorithm == BFS) {
        directions = directions_4_way;
        num_directions = 4;
    }
    else if (algorithm == DFS_ALGORITHM) {
        directions = directions_4_way;
        num_directions = 4;
    }
    else {
        directions = directions_8_way;
        num_directions = 8;
    }

    graph.resize(rows * cols);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (img.at<uchar>(y, x) == 255) {
                int current_node = y * cols + x;
                for (int i = 0; i < num_directions; ++i) {
                    int new_x = x + directions[i][0];
                    int new_y = y + directions[i][1];
                    if (new_x >= 0 && new_x < cols && new_y >= 0 && new_y < rows) {
                        if (img.at<uchar>(new_y, new_x) == 255) {
                            int neighbor_node = new_y * cols + new_x;
                            graph[current_node].push_back(neighbor_node);
                        }
                    }
                }
            }
        }
    }
}


int manhattanDistanceHeuristic(int start, int node, int goal, int cols) {
    int goal_x = goal % cols;
    int goal_y = goal / cols;
    int node_x = node % cols;
    int node_y = node / cols;

    int heuristic = 0;
    heuristic = std::abs(goal_x - node_x) + std::abs(goal_y - node_y);

    return heuristic;
}

int euclideanDistanceHeuristic(int start, int node, int goal, int cols) {
    int goal_x = goal % cols;
    int goal_y = goal / cols;
    int node_x = node % cols;
    int node_y = node / cols;
    int heuristic = static_cast<int>(std::sqrt(std::pow(goal_x - node_x, 2) + std::pow(goal_y - node_y, 2)));

    int start_x = start % cols;
    int start_y = start / cols;

    int dx_start = start_x - goal_x;
    int dy_start = start_y - goal_y;

    int dx_node = node_x - goal_x;
    int dy_node = node_y - goal_y;

    int cross_product = abs((dx_start * dy_node) - (dy_start * dx_node));

    heuristic += cross_product * 0.001;
    return heuristic;
}


void convertWorldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y, double room_width, double room_height, int image_width, int image_height) {
    pixel_x = static_cast<int>(world_x * image_width / room_width);
    pixel_y = static_cast<int>(image_height - (world_y * image_height / room_height));
}

void convertPixelToWorld(int pixel_x, int pixel_y, double& world_x, double& world_y, double room_width, double room_height, int image_width, int image_height) {
    world_x = static_cast<double>(pixel_x) * (room_width / image_width);
    world_y = room_height - static_cast<double>(pixel_y) * (room_height / image_height);
}

void drawPathOnMap(const std::vector<waypointType>& valid_waypoints, const cv::Mat& img, const std::string& output_path) {
    cv::Mat output_image;

    cvtColor(img, output_image, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < valid_waypoints.size(); i++) {
        cv::drawMarker(output_image, cv::Point(valid_waypoints[i].x, valid_waypoints[i].y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    }

   for (int i = 0; i < valid_path.size(); i++) {
      cv::drawMarker(output_image, cv::Point(valid_path[i].x, valid_path[i].y), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 5, 1);
   }

    cv::imwrite(output_path, output_image);
}

double pathOrientation(std::vector<pointType>& valid_path, int i) {
    double delta_x;
    double delta_y;
    double orientation = 0;

    if (valid_path.size() < 3) {
        orientation = 0;
    } else if (i == 0) {
        delta_x = valid_path[i + 1].x - valid_path[i].x;
        delta_y = valid_path[i + 1].y - valid_path[i].y;
    } else if (i == valid_path.size() - 1) {
        delta_x = valid_path[i].x - valid_path[i - 1].x;
        delta_y = valid_path[i].y - valid_path[i - 1].y;
    } else {
        delta_x = valid_path[i + 1].x - valid_path[i - 1].x;
        delta_y = valid_path[i + 1].y - valid_path[i - 1].y;
    }

    orientation = std::atan2(-delta_y, delta_x);

    return orientation;
}


void computeWaypoints(std::vector<int>& robot_path, std::vector<pointType>& valid_path, std::vector<waypointType>& valid_waypoints, int suppression_factor, double curvature_angle) {
    valid_path.clear();
    valid_waypoints.clear();

    if (robot_path.size() < 2) {
        return;
    }

    for (int i = 0; i < robot_path.size(); i++) {
        int x = robot_path[i] % image_width;
        int y = robot_path[i] / image_width;
        valid_path.push_back({x, y});
    }

    if (valid_path.size() < 3) {
        if (valid_path.size() >= 1) {
            double orientation = (valid_path.size() > 1) ?
                atan2(valid_path[1].y - valid_path[0].y, valid_path[1].x - valid_path[0].x) : 0.0;
            valid_waypoints.push_back({valid_path[0].x, valid_path[0].y, orientation});
        }
        if (valid_path.size() >= 2) {
            double orientation = atan2(valid_path[valid_path.size()-1].y - valid_path[valid_path.size()-2].y,
                                     valid_path[valid_path.size()-1].x - valid_path[valid_path.size()-2].x);
            valid_waypoints.push_back({valid_path.back().x, valid_path.back().y, orientation});
        }
        return;
    }

    const double MIN_TURNING_RADIUS = 30.0;
    const double WAYPOINT_SPACING = std::max(40.0, (double)suppression_factor * 2.0);
    const double DIRECTION_CHANGE_THRESHOLD = curvature_angle * 0.7;
    const int LOOKAHEAD_DISTANCE = 15;

    std::vector<waypointType> candidate_waypoints;

    double start_orientation = atan2(valid_path[1].y - valid_path[0].y, valid_path[1].x - valid_path[0].x);
    candidate_waypoints.push_back({valid_path[0].x, valid_path[0].y, start_orientation});

    for (int i = LOOKAHEAD_DISTANCE; i < valid_path.size() - LOOKAHEAD_DISTANCE; i++) {
        std::vector<double> segment_directions;

        for (int j = -LOOKAHEAD_DISTANCE/2; j <= LOOKAHEAD_DISTANCE/2; j += 3) {
            int idx1 = std::max(0, std::min((int)valid_path.size()-1, i + j - 3));
            int idx2 = std::max(0, std::min((int)valid_path.size()-1, i + j + 3));

            if (idx2 > idx1) {
                double dir = atan2(valid_path[idx2].y - valid_path[idx1].y,
                                 valid_path[idx2].x - valid_path[idx1].x);
                segment_directions.push_back(dir);
            }
        }

        if (segment_directions.size() < 2) continue;

        double max_direction_change = 0.0;
        for (int j = 1; j < segment_directions.size(); j++) {
            double angle_diff = segment_directions[j] - segment_directions[j-1];

            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            max_direction_change = std::max(max_direction_change, std::abs(angle_diff));
        }

        bool significant_turn = max_direction_change > DIRECTION_CHANGE_THRESHOLD;

        bool sufficient_distance = true;
        if (!candidate_waypoints.empty()) {
            double dist = sqrt(pow(valid_path[i].x - candidate_waypoints.back().x, 2) +
                             pow(valid_path[i].y - candidate_waypoints.back().y, 2));
            sufficient_distance = (dist >= WAYPOINT_SPACING);
        }

        double path_complexity = 0.0;
        for (int j = i; j < std::min(i + LOOKAHEAD_DISTANCE*2, (int)valid_path.size()-1); j++) {
            if (j < segment_directions.size()-1) {
                path_complexity += std::abs(segment_directions[j+1] - segment_directions[j]);
            }
        }

        if ((significant_turn && sufficient_distance) ||
            (sufficient_distance && path_complexity > DIRECTION_CHANGE_THRESHOLD * 2)) {
            double approach_angle = calculateOptimalApproachAngle(valid_path, i, LOOKAHEAD_DISTANCE);
            candidate_waypoints.push_back({valid_path[i].x, valid_path[i].y, approach_angle});
        }
    }

    int last_idx = valid_path.size() - 1;
    double end_orientation = (valid_path.size() > 1) ?
        atan2(valid_path[last_idx].y - valid_path[last_idx-1].y,
              valid_path[last_idx].x - valid_path[last_idx-1].x) : 0.0;
    candidate_waypoints.push_back({valid_path[last_idx].x, valid_path[last_idx].y, end_orientation});

    valid_waypoints = optimizeWaypointSequence(candidate_waypoints, MIN_TURNING_RADIUS);

    if (valid_waypoints.size() < 2 && valid_path.size() >= 2) {
        valid_waypoints.clear();
        valid_waypoints.push_back({valid_path[0].x, valid_path[0].y, start_orientation});
        valid_waypoints.push_back({valid_path.back().x, valid_path.back().y, end_orientation});
    }
}

double calculateOptimalApproachAngle(const std::vector<pointType>& path, int current_idx, int lookahead) {
    if (current_idx >= path.size()) return 0.0;

    int future_idx = std::min(current_idx + lookahead, (int)path.size() - 1);
    int past_idx = std::max(current_idx - lookahead/2, 0);

    double future_weight = 0.7;
    double past_weight = 0.3;

    double future_angle = 0.0;
    double past_angle = 0.0;

    if (future_idx > current_idx) {
        future_angle = atan2(path[future_idx].y - path[current_idx].y,
                           path[future_idx].x - path[current_idx].x);
    }

    if (current_idx > past_idx) {
        past_angle = atan2(path[current_idx].y - path[past_idx].y,
                         path[current_idx].x - path[past_idx].x);
    }

    double optimal_angle = future_weight * future_angle + past_weight * past_angle;

    return optimal_angle;
}

std::vector<waypointType> optimizeWaypointSequence(const std::vector<waypointType>& candidates, double min_turning_radius) {
    if (candidates.size() <= 2) return candidates;

    std::vector<waypointType> optimized;
    optimized.push_back(candidates[0]);

    for (int i = 1; i < candidates.size() - 1; i++) {
        const auto& prev = optimized.back();
        const auto& current = candidates[i];
        const auto& next = candidates[i + 1];

        double dist_to_prev = sqrt(pow(current.x - prev.x, 2) + pow(current.y - prev.y, 2));
        double dist_to_next = sqrt(pow(next.x - current.x, 2) + pow(next.y - current.y, 2));

        double angle_in = atan2(current.y - prev.y, current.x - prev.x);
        double angle_out = atan2(next.y - current.y, next.x - current.x);
        double turn_angle = angle_out - angle_in;

        while (turn_angle > M_PI) turn_angle -= 2 * M_PI;
        while (turn_angle < -M_PI) turn_angle += 2 * M_PI;

        if (std::abs(turn_angle) > 0.1 && dist_to_prev > min_turning_radius * 0.5) {
            waypointType adjusted = current;
            adjusted.theta = (angle_in + angle_out) / 2.0;
            optimized.push_back(adjusted);
        }
    }

    optimized.push_back(candidates.back());
    return optimized;
}


void printWaypoints(std::vector<waypointType>& valid_waypoints, double room_width, double room_height, int image_width, int image_height){
    for(int i = 0; i < valid_waypoints.size(); i++){
        double world_x;
        double world_y;
        convertPixelToWorld(valid_waypoints[i].x, valid_waypoints[i].y, world_x, world_y, room_width, room_height, image_width, image_height);
        ROS_INFO("Waypoint %d: x = %.2f, y = %.2f, theta = %.2f\n", i, world_x, world_y, valid_waypoints[i].theta);
    }
}

std::vector<int> astar(int start, int goal, const std::vector<std::vector<int>>& graph, int cols, int heuristic_type) {
    auto heuristic = (heuristic_type == MANHATTAN_HEURISTIC) ? manhattanDistanceHeuristic : euclideanDistanceHeuristic;

    std::vector<int> dist(graph.size(), INT_MAX);
    std::vector<int> came_from(graph.size(), -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0 + heuristic(start, start, goal, cols), start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            int new_dist = dist[current] + 1;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                came_from[neighbor] = current;
                pq.push({new_dist + heuristic(start, neighbor, goal, cols), neighbor});
            }
        }
    }
    return {};
}

std::vector<int> bfs(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::vector<int> dist(graph.size(), -1);
    std::vector<int> came_from(graph.size(), -1);
    std::queue<int> q;

    dist[start] = 0;
    q.push(start);

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            if (dist[neighbor] == -1) {
                dist[neighbor] = dist[current] + 1;
                came_from[neighbor] = current;
                q.push(neighbor);
            }
        }
    }
    return {};
}

std::vector<int> dijkstra(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::vector<int> dist(graph.size(), INT_MAX);
    std::vector<int> came_from(graph.size(), -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            int new_dist = dist[current] + 1;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                came_from[neighbor] = current;
                pq.push({new_dist, neighbor});
            }
        }
    }
    return {};
}

std::vector<int> dfs(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::vector<bool> visited(graph.size(), false);
    std::vector<int> came_from(graph.size(), -1);
    std::stack<int> s;

    s.push(start);

    while (!s.empty()) {
        int current = s.top();
        s.pop();

        if (visited[current]) continue;
        visited[current] = true;

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            if (!visited[neighbor]) {
                came_from[neighbor] = current;
                s.push(neighbor);
            }
        }
    }
    return {};
}


int planRobotPath(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug){
    int start_x_pixel;
    int start_y_pixel;
    int goal_x_pixel;
    int goal_y_pixel;

    convertWorldToPixel(start_x, start_y, start_x_pixel, start_y_pixel, room_width, room_height, image_width, image_height);
    convertWorldToPixel(goal_x, goal_y, goal_x_pixel, goal_y_pixel, room_width, room_height, image_width, image_height);

    int start_node = start_y_pixel * image_width + start_x_pixel;
    int goal_node = goal_y_pixel * image_width + goal_x_pixel;

    if(start_node < 0 || start_node >= graph.size() || goal_node < 0 || goal_node >= graph.size()){
        ROS_ERROR("Invalid start or goal node");
        return 0;
    }

    switch(pathPlanningAlgorithm){
        case BFS_ALGORITHM:
            robot_path = bfs(start_node, goal_node, graph);
            break;
        case DIJKSTRA_ALGORITHM:
            robot_path = dijkstra(start_node, goal_node, graph);
            break;
        case ASTAR_ALGORITHM:
            robot_path = astar(start_node, goal_node, graph, image_width, EUCLIDEAN_HEURISTIC);
            break;
        case DFS_ALGORITHM:
            robot_path = dfs(start_node, goal_node, graph);
            break;
        default:
            robot_path = astar(start_node, goal_node, graph, image_width, EUCLIDEAN_HEURISTIC);
            break;
    }

    if(robot_path.size() == 0){
        ROS_ERROR("No path found from start to goal");
        return 0;
    }

    computeWaypoints(robot_path, valid_path, valid_waypoints, suppression_factor, CURVATURE_THRESHOLD);

    if(debug){
        printWaypoints(valid_waypoints, room_width, room_height, image_width, image_height);
    }

    return 1;
}


int navigateToGoal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug){
   ros::Rate rate(PUBLISH_RATE);

   int path_found = 0;
   int robot_moved = 0;
   path_found = planRobotPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, pathPlanningAlgorithm, mapImage, configurationSpaceImage, debug);
   if (path_found){
      if(debug){
         printf("Path found from (%5.3f %5.3f %5.3f) to (%5.3f %5.3f %5.3f)\n", start_x, start_y, degrees(start_theta), goal_x, goal_y, degrees(goal_theta));
      }
      robot_moved = moveRobot(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, velocity_publisher, rate, debug);
   }
   return robot_moved;
}

/*
 * Action-based navigation function with feedback support
 */
int navigateToGoalWithFeedback(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta,
                               int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug,
                               actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server){
   ros::Rate rate(PUBLISH_RATE);

   int path_found = 0;
   int robot_moved = 0;
   path_found = planRobotPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, pathPlanningAlgorithm, mapImage, configurationSpaceImage, debug);
   if (path_found){
      if(debug){
         printf("Path found from (%5.3f %5.3f %5.3f) to (%5.3f %5.3f %5.3f)\n", start_x, start_y, degrees(start_theta), goal_x, goal_y, degrees(goal_theta));
      }
      robot_moved = moveRobotWithFeedback(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, velocity_publisher, rate, debug, action_server);
   }
   return robot_moved;
}


ControlClientPtr createClient(const std::string& topic_name) {
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 10;

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topic_name.c_str());
    }
    return nullptr;
}


int goToHome(std::string actuator, std::string topics_filename, bool debug){
    std::vector<double> actuator_state;
    std::vector<double> actuator_home_position;
    ControlClientPtr actuator_client;
    std::vector<std::string> actuator_joint_names;
    std::string actuator_topic;
    int number_of_joints;
    double home_duration;

    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    if(extractTopic(actuator, topics_filename, &actuator_topic)){
        return 0;
    }

    home_duration = 0.5;

    if(actuator == "Leg"){
        actuator_home_position = leg_home_position;
        actuator_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
        actuator_client = createClient(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }
    else if(actuator == "Head"){
        actuator_home_position = head_home_position;
        actuator_joint_names = {"HeadPitch", "HeadYaw"};
        actuator_client = createClient(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }

    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

   moveToPosition(actuator_client, actuator_joint_names, home_duration, open_hand, hand, hand_topic, "home", actuator_home_position);

    return 1;
}


void setOdometryPose(double x, double y, double theta)
{
   bool debug = false;

   initial_x = x;
   initial_y = y;
   initial_theta = theta;

   adjustment_x = initial_x - odom_x;
   adjustment_y = initial_y - odom_y;
   adjustment_theta = initial_theta - odom_theta;

   if (debug)
   {
      printf("initial_x, initial_y, initial_theta: %f, %f, %f\n", initial_x, initial_y, initial_theta);
      printf("odom_x, odom_y, odom_theta:          %f, %f, %f\n", odom_x, odom_y, odom_theta);
      printf("adjustment:                          %f, %f, %f\n", adjustment_x, adjustment_y, adjustment_theta);
   }
}


int moveRobot(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, ros::Publisher velocity_publisher, ros::Rate rate, bool debug) {

    if (locomotionParameterData.robot_available == false) {
        printf("No physical robot present\n");
        return 0;
    }

    if (valid_waypoints.size() == 0) {
        printf("No waypoints available for navigation\n");
        return 0;
    }

    return executeSmoothWaypointNavigation(start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
                                         velocity_publisher, rate, debug);
}

/*
 * Action-based move robot function with feedback support
 */
int moveRobotWithFeedback(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta,
                          ros::Publisher velocity_publisher, ros::Rate rate, bool debug,
                          actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server) {

    if (locomotionParameterData.robot_available == false) {
        printf("No physical robot present\n");
        return 0;
    }

    if (valid_waypoints.size() == 0) {
        printf("No waypoints available for navigation\n");
        return 0;
    }

    return executeSmoothWaypointNavigationWithFeedback(start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
                                                       velocity_publisher, rate, debug, action_server);
}


int executeSmoothWaypointNavigation(double start_x, double start_y, double start_theta,
                                   double goal_x, double goal_y, double goal_theta,
                                   ros::Publisher velocity_publisher, ros::Rate rate, bool debug) {

    geometry_msgs::Twist msg;

    const double WAYPOINT_APPROACH_RADIUS = 0.25;
    const double WAYPOINT_CAPTURE_RADIUS = 0.15;
    const double LOOKAHEAD_DISTANCE = 0.4;
    const double MAX_LINEAR_VEL = locomotionParameterData.max_linear_velocity * 0.9;
    const double MAX_ANGULAR_VEL = locomotionParameterData.max_angular_velocity * 0.9;
    const double MIN_LINEAR_VEL = locomotionParameterData.min_linear_velocity;
    const double MIN_ANGULAR_VEL = locomotionParameterData.min_angular_velocity;

    int current_waypoint_idx = 0;
    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    bool navigation_complete = false;

    auto getWaypointWorldCoords = [&](int idx) -> std::tuple<double, double, double> {
        if (idx >= valid_waypoints.size()) return std::make_tuple(goal_x, goal_y, goal_theta);

        double world_x, world_y;
        convertPixelToWorld(valid_waypoints[idx].x, valid_waypoints[idx].y, world_x, world_y,
                          room_width, room_height, image_width, image_height);
        return std::make_tuple(world_x, world_y, valid_waypoints[idx].theta);
    };

    if (debug) {
        printf("Starting smooth waypoint navigation with %zu waypoints\n", valid_waypoints.size());
    }

    while (!navigation_complete && ros::ok()) {
        ros::spinOnce();

        auto [target_x, target_y, target_theta] = getWaypointWorldCoords(current_waypoint_idx);

        double distance_to_waypoint = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

        if (distance_to_waypoint < WAYPOINT_CAPTURE_RADIUS) {
            current_waypoint_idx++;
            if (current_waypoint_idx >= valid_waypoints.size()) {
                navigation_complete = true;
                break;
            }

            if (debug) {
                printf("Advanced to waypoint %d\n", current_waypoint_idx);
            }
            continue;
        }

        double lookahead_x = target_x;
        double lookahead_y = target_y;
        double lookahead_weight = 0.0;

        if (current_waypoint_idx + 1 < valid_waypoints.size() && distance_to_waypoint < WAYPOINT_APPROACH_RADIUS) {
            auto [next_x, next_y, next_theta] = getWaypointWorldCoords(current_waypoint_idx + 1);

            lookahead_weight = 1.0 - (distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            lookahead_weight = std::min(0.6, lookahead_weight);

            lookahead_x = target_x * (1.0 - lookahead_weight) + next_x * lookahead_weight;
            lookahead_y = target_y * (1.0 - lookahead_weight) + next_y * lookahead_weight;
        }

        double goal_direction = atan2(lookahead_y - current_y, lookahead_x - current_x);
        double angle_error = goal_direction - current_theta;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        double target_linear_vel = MAX_LINEAR_VEL;

        double turn_factor = 1.0 - std::min(1.0, std::abs(angle_error) / (M_PI/3));
        target_linear_vel *= (0.4 + 0.6 * turn_factor);

        if (lookahead_weight < 0.3) {
            double approach_factor = std::min(1.0, distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            target_linear_vel *= (0.5 + 0.5 * approach_factor);
        }

        target_linear_vel = std::max(target_linear_vel, MIN_LINEAR_VEL);

        double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error;
        target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);

        if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > 0.05) {
            target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
        }

        double accel_limit = 0.15;
        double angular_accel_limit = 0.2;

        double linear_vel_diff = target_linear_vel - current_linear_velocity;
        if (std::abs(linear_vel_diff) > accel_limit) {
            current_linear_velocity += accel_limit * ((linear_vel_diff > 0) ? 1 : -1);
        } else {
            current_linear_velocity = target_linear_vel;
        }

        double angular_vel_diff = target_angular_vel - current_angular_velocity;
        if (std::abs(angular_vel_diff) > angular_accel_limit) {
            current_angular_velocity += angular_accel_limit * ((angular_vel_diff > 0) ? 1 : -1);
        } else {
            current_angular_velocity = target_angular_vel;
        }

        msg.linear.x = current_linear_velocity;
        msg.angular.z = current_angular_velocity;
        velocity_publisher.publish(msg);

        rate.sleep();
    }

    if (navigation_complete) {
        if (debug) {
            printf("Executing final precision navigation to goal\n");
        }

        goToPoseDQ(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate);
    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_publisher.publish(msg);

    if (debug) {
        printf("Smooth waypoint navigation completed successfully\n");
    }

    return 1;
}


/*
 * Action-based smooth waypoint navigation with preemption and feedback support
 */
int executeSmoothWaypointNavigationWithFeedback(double start_x, double start_y, double start_theta,
                                                double goal_x, double goal_y, double goal_theta,
                                                ros::Publisher velocity_publisher, ros::Rate rate, bool debug,
                                                actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server) {

    geometry_msgs::Twist msg;
    cssr_system::setGoalFeedback feedback;

    const double WAYPOINT_APPROACH_RADIUS = 0.25;
    const double WAYPOINT_CAPTURE_RADIUS = 0.15;
    const double LOOKAHEAD_DISTANCE = 0.4;
    const double MAX_LINEAR_VEL = locomotionParameterData.max_linear_velocity * 0.9;
    const double MAX_ANGULAR_VEL = locomotionParameterData.max_angular_velocity * 0.9;
    const double MIN_LINEAR_VEL = locomotionParameterData.min_linear_velocity;
    const double MIN_ANGULAR_VEL = locomotionParameterData.min_angular_velocity;

    int current_waypoint_idx = 0;
    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    bool navigation_complete = false;

    // Feedback timing
    ros::Time last_feedback_time = ros::Time::now();
    double feedback_interval = 1.0 / FEEDBACK_RATE;

    auto getWaypointWorldCoords = [&](int idx) -> std::tuple<double, double, double> {
        if (idx >= valid_waypoints.size()) return std::make_tuple(goal_x, goal_y, goal_theta);

        double world_x, world_y;
        convertPixelToWorld(valid_waypoints[idx].x, valid_waypoints[idx].y, world_x, world_y,
                          room_width, room_height, image_width, image_height);
        return std::make_tuple(world_x, world_y, valid_waypoints[idx].theta);
    };

    if (debug) {
        printf("Starting smooth waypoint navigation with %zu waypoints\n", valid_waypoints.size());
    }

    while (!navigation_complete && ros::ok()) {
        ros::spinOnce();

        // Check for preemption
        if (action_server->isPreemptRequested()) {
            ROS_INFO("Navigation preempted");
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            velocity_publisher.publish(msg);
            return 0;
        }

        auto [target_x, target_y, target_theta] = getWaypointWorldCoords(current_waypoint_idx);

        double distance_to_waypoint = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        double distance_to_goal = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));

        // Publish feedback at the specified rate
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_feedback_time).toSec() >= feedback_interval) {
            feedback.distance_remaining = distance_to_goal;
            feedback.current_x = current_x;
            feedback.current_y = current_y;
            feedback.current_theta = degrees(current_theta);
            action_server->publishFeedback(feedback);
            last_feedback_time = current_time;
        }

        if (distance_to_waypoint < WAYPOINT_CAPTURE_RADIUS) {
            current_waypoint_idx++;
            if (current_waypoint_idx >= valid_waypoints.size()) {
                navigation_complete = true;
                break;
            }

            if (debug) {
                printf("Advanced to waypoint %d\n", current_waypoint_idx);
            }
            continue;
        }

        double lookahead_x = target_x;
        double lookahead_y = target_y;
        double lookahead_weight = 0.0;

        if (current_waypoint_idx + 1 < valid_waypoints.size() && distance_to_waypoint < WAYPOINT_APPROACH_RADIUS) {
            auto [next_x, next_y, next_theta] = getWaypointWorldCoords(current_waypoint_idx + 1);

            lookahead_weight = 1.0 - (distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            lookahead_weight = std::min(0.6, lookahead_weight);

            lookahead_x = target_x * (1.0 - lookahead_weight) + next_x * lookahead_weight;
            lookahead_y = target_y * (1.0 - lookahead_weight) + next_y * lookahead_weight;
        }

        double goal_direction = atan2(lookahead_y - current_y, lookahead_x - current_x);
        double angle_error = goal_direction - current_theta;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        double target_linear_vel = MAX_LINEAR_VEL;

        double turn_factor = 1.0 - std::min(1.0, std::abs(angle_error) / (M_PI/3));
        target_linear_vel *= (0.4 + 0.6 * turn_factor);

        if (lookahead_weight < 0.3) {
            double approach_factor = std::min(1.0, distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            target_linear_vel *= (0.5 + 0.5 * approach_factor);
        }

        target_linear_vel = std::max(target_linear_vel, MIN_LINEAR_VEL);

        double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error;
        target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);

        if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > 0.05) {
            target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
        }

        double accel_limit = 0.15;
        double angular_accel_limit = 0.2;

        double linear_vel_diff = target_linear_vel - current_linear_velocity;
        if (std::abs(linear_vel_diff) > accel_limit) {
            current_linear_velocity += accel_limit * ((linear_vel_diff > 0) ? 1 : -1);
        } else {
            current_linear_velocity = target_linear_vel;
        }

        double angular_vel_diff = target_angular_vel - current_angular_velocity;
        if (std::abs(angular_vel_diff) > angular_accel_limit) {
            current_angular_velocity += angular_accel_limit * ((angular_vel_diff > 0) ? 1 : -1);
        } else {
            current_angular_velocity = target_angular_vel;
        }

        msg.linear.x = current_linear_velocity;
        msg.angular.z = current_angular_velocity;
        velocity_publisher.publish(msg);

        rate.sleep();
    }

    if (navigation_complete) {
        if (debug) {
            printf("Executing final precision navigation to goal\n");
        }

        goToPoseDQ(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate);
    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_publisher.publish(msg);

    // Final feedback
    feedback.distance_remaining = 0.0;
    feedback.current_x = current_x;
    feedback.current_y = current_y;
    feedback.current_theta = degrees(current_theta);
    action_server->publishFeedback(feedback);

    if (debug) {
        printf("Smooth waypoint navigation completed successfully\n");
    }

    return 1;
}


void goToPoseDQ(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate) {

    bool debug = false;
    geometry_msgs::Twist msg;

    double goal_x = x;
    double goal_y = y;
    double goal_theta = theta;

    const double APPROACH_DISTANCE_THRESHOLD = 0.15;
    const double POSITION_TOLERANCE = locomotionParameterData.position_tolerance_goal;
    const double ANGLE_TOLERANCE = locomotionParameterData.angle_tolerance_orienting;
    const double MAX_LINEAR_VEL = locomotionParameterData.max_linear_velocity * 0.8;
    const double MAX_ANGULAR_VEL = locomotionParameterData.max_angular_velocity * 0.8;
    const double MIN_LINEAR_VEL = locomotionParameterData.min_linear_velocity;
    const double MIN_ANGULAR_VEL = locomotionParameterData.min_angular_velocity;

    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    double position_error = 0.0;
    double angle_error = 0.0;

    enum MotionPhase { APPROACHING, FINE_POSITIONING, FINAL_ORIENTATION, COMPLETED };
    MotionPhase phase = APPROACHING;

    while (phase != COMPLETED && ros::ok()) {
        ros::spinOnce();

        position_error = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));

        if (phase == APPROACHING || phase == FINE_POSITIONING) {
            double goal_direction = atan2(goal_y - current_y, goal_x - current_x);
            angle_error = goal_direction - current_theta;

            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;

            if (position_error <= POSITION_TOLERANCE) {
                phase = FINAL_ORIENTATION;
                continue;
            }

            double distance_scale = std::min(1.0, position_error / APPROACH_DISTANCE_THRESHOLD);
            double angular_scale = 1.0 - std::min(1.0, std::abs(angle_error) / (M_PI/4));

            double target_linear_vel = MAX_LINEAR_VEL * distance_scale * angular_scale;
            target_linear_vel = std::max(target_linear_vel, MIN_LINEAR_VEL);

            double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error;
            target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);
            if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > 0.05) {
                target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
            }

            double linear_accel = 0.1;
            double angular_accel = 0.15;

            if (target_linear_vel > current_linear_velocity) {
                current_linear_velocity += linear_accel;
                current_linear_velocity = std::min(current_linear_velocity, target_linear_vel);
            } else {
                current_linear_velocity -= linear_accel;
                current_linear_velocity = std::max(current_linear_velocity, target_linear_vel);
            }

            if (target_angular_vel > current_angular_velocity) {
                current_angular_velocity += angular_accel;
                current_angular_velocity = std::min(current_angular_velocity, target_angular_vel);
            } else {
                current_angular_velocity -= angular_accel;
                current_angular_velocity = std::max(current_angular_velocity, target_angular_vel);
            }

            msg.linear.x = current_linear_velocity;
            msg.angular.z = current_angular_velocity;

        } else if (phase == FINAL_ORIENTATION) {
            angle_error = theta - current_theta;

            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;

            if (std::abs(angle_error) <= ANGLE_TOLERANCE) {
                phase = COMPLETED;
                continue;
            }

            current_linear_velocity *= 0.9;
            if (current_linear_velocity < 0.01) current_linear_velocity = 0.0;

            double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error * 0.8;
            target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);

            if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > ANGLE_TOLERANCE) {
                target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
            }

            if (std::abs(angle_error) < 0.2) {
                target_angular_vel *= (std::abs(angle_error) / 0.2);
            }

            current_angular_velocity = target_angular_vel;

            msg.linear.x = current_linear_velocity;
            msg.angular.z = current_angular_velocity;
        }

        pub.publish(msg);
        rate.sleep();
    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
}


void goToPoseMIMO(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
{
   bool debug = false;

   geometry_msgs::Twist msg;

   double goal_x = x;
   double goal_y = y;
   double goal_theta = theta;

   double goal_direction;
   double position_error;
   double angle_error;
   double angular_velocity;
   double linear_velocity;

   static double current_linear_velocity = 0;

   int number_of_ramp_up_steps = 20;

   position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

   while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok())
   {
     ros::spinOnce();

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     if (waypoints)
     {
         linear_velocity = locomotionParameterData.max_linear_velocity / 2;
     }
     else
     {
         linear_velocity = locomotionParameterData.position_gain_mimo * position_error;
     }

     if (linear_velocity < locomotionParameterData.min_linear_velocity)
         msg.linear.x = locomotionParameterData.min_linear_velocity;
     else if (linear_velocity > locomotionParameterData.max_linear_velocity)
         msg.linear.x = locomotionParameterData.max_linear_velocity;

     if (current_linear_velocity == 0)
     {
         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
             msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
             msg.angular.z = 0;
             pub.publish(msg);
             rate.sleep();
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;
     pub.publish(msg);
     rate.sleep();
   }

   if (!waypoints)
   {
       angle_error = goal_theta - current_theta;

       while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok())
       {
         ros::spinOnce();

         angle_error = goal_theta - current_theta;

         current_linear_velocity = 0;
         msg.linear.x = current_linear_velocity;

         angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
             msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
             msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
             msg.angular.z = angular_velocity;

         pub.publish(msg);
         rate.sleep();
       }

       current_linear_velocity = 0;
       msg.linear.x = current_linear_velocity;
       angular_velocity = 0;
       msg.angular.z = angular_velocity;
       pub.publish(msg);
   }
}


void saveWaypointMap(vector<int> compressionParams, Mat mapImageLarge, string fileName){
    char path_and_input_filename[MAX_FILENAME_LENGTH] = "";
    std::string navigation_pathway_filename = fileName;

    strcpy(path_and_input_filename, packagedir.c_str());
    strcat(path_and_input_filename, "/robotNavigation/data/");
    strcat(path_and_input_filename, navigation_pathway_filename.c_str());

    cv::imwrite(path_and_input_filename, mapImageLarge, compressionParams);
    printf("Navigation pathway image is saved in %s\n", path_and_input_filename);
}


void stabilizeWaist()
{
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("HipRoll");
    msg.joint_angles.push_back(0.1);
    msg.joint_names.push_back("HipPitch");
    msg.joint_angles.push_back(-0.1);
    msg.joint_names.push_back("KneePitch");
    msg.joint_angles.push_back(0.0);
    msg.speed = 0.2;
    msg.relative = 0;

    navigation_pelvis_publisher.publish(msg);
}

void stabilizeWaistContinuously()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        stabilizeWaist();
        rate.sleep();
    }
}


double roundFloatingPoint(double value, int decimal_places){
   double rounded_value = 0.0;
   double multiplier = std::pow(10.0, decimal_places);
   rounded_value = std::roundf(value * multiplier) / multiplier;
   return rounded_value;
}

double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI;
    return degrees;
}

double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI);
    return radians;
}


void promptAndExit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

void promptAndContinue(){
    printf("Press X to quit or Press any key to continue...\n");
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}

int signnum(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}


void moveRobotActuatorsToDefault(){
    double gesture_duration = 1.0;

    std::string head_topic;
    head_topic = "/pepper_dcm/Head_controller/follow_joint_trajectory";
    ControlClientPtr head_client = createClient(head_topic);

    if(head_client == nullptr){
        ROS_ERROR("Error creating action client for head controller");
        return;
    }
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};
    std::vector<double> head_position = {0.0, 0.0};

    std::string left_arm_topic;
    left_arm_topic = "/pepper_dcm/LeftArm_controller/follow_joint_trajectory";
    ControlClientPtr left_arm_client = createClient(left_arm_topic);

    if(left_arm_client == nullptr){
        ROS_ERROR("Error creating action client for left arm controller");
        return;
    }

    std::vector<std::string> left_arm_joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};
    std::vector<double> left_arm_position = {1.5523885488510132, 0.13959217071533203, -1.2087767124176025, -0.4540584087371826, -0.15497589111328125};

    std::string right_arm_topic;
    right_arm_topic = "/pepper_dcm/RightArm_controller/follow_joint_trajectory";
    ControlClientPtr right_arm_client = createClient(right_arm_topic);

    if(right_arm_client == nullptr){
        ROS_ERROR("Error creating action client for right arm controller");
        return;
    }

    std::vector<std::string> right_arm_joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};
    std::vector<double> right_arm_position = {1.5447185039520264, -0.1441941261291504, 1.2133787870407104, 0.44945645332336426, 0.1487560272216797};

    std::string leg_topic;
    leg_topic = "/pepper_dcm/Pelvis_controller/follow_joint_trajectory";
    ControlClientPtr leg_client = createClient(leg_topic);

    if(leg_client == nullptr){
        ROS_ERROR("Error creating action client for leg controller");
        return;
    }

    std::vector<std::string> leg_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> leg_position = {0.0, 0.0, 0.0};

    moveToPosition(head_client, head_joint_names, head_position,
                     left_arm_client, left_arm_joint_names, left_arm_position,
                     right_arm_client, right_arm_joint_names, right_arm_position,
                     leg_client, leg_joint_names, leg_position,
                     gesture_duration);
}

void moveToPosition(ControlClientPtr& head_client, const std::vector<std::string>& head_joint_names, std::vector<double> head_positions,
                        ControlClientPtr& left_arm_client, const std::vector<std::string>& left_arm_joint_names, std::vector<double> left_arm_positions,
                        ControlClientPtr& right_arm_client, const std::vector<std::string>& right_arm_joint_names, std::vector<double> right_arm_positions,
                        ControlClientPtr& leg_client, const std::vector<std::string>& leg_joint_names, std::vector<double> leg_positions,
                        double duration){
    control_msgs::FollowJointTrajectoryGoal head_goal;
    trajectory_msgs::JointTrajectory& head_trajectory = head_goal.trajectory;
    head_trajectory.joint_names = head_joint_names;
    head_trajectory.points.resize(1);
    head_trajectory.points[0].positions = head_positions;
    head_trajectory.points[0].time_from_start = ros::Duration(duration);

    control_msgs::FollowJointTrajectoryGoal left_arm_goal;
    trajectory_msgs::JointTrajectory& left_arm_trajectory = left_arm_goal.trajectory;
    left_arm_trajectory.joint_names = left_arm_joint_names;
    left_arm_trajectory.points.resize(1);
    left_arm_trajectory.points[0].positions = left_arm_positions;
    left_arm_trajectory.points[0].time_from_start = ros::Duration(duration);

    control_msgs::FollowJointTrajectoryGoal right_arm_goal;
    trajectory_msgs::JointTrajectory& right_arm_trajectory = right_arm_goal.trajectory;
    right_arm_trajectory.joint_names = right_arm_joint_names;
    right_arm_trajectory.points.resize(1);
    right_arm_trajectory.points[0].positions = right_arm_positions;
    right_arm_trajectory.points[0].time_from_start = ros::Duration(duration);

    control_msgs::FollowJointTrajectoryGoal leg_goal;
    trajectory_msgs::JointTrajectory& leg_trajectory = leg_goal.trajectory;
    leg_trajectory.joint_names = leg_joint_names;
    leg_trajectory.points.resize(1);
    leg_trajectory.points[0].positions = leg_positions;
    leg_trajectory.points[0].time_from_start = ros::Duration(duration);

   left_arm_client->sendGoal(left_arm_goal);
   right_arm_client->sendGoal(right_arm_goal);
   leg_client->sendGoal(leg_goal);

   left_arm_client->waitForResult(ros::Duration(duration));
   right_arm_client->waitForResult(ros::Duration(duration));
   leg_client->waitForResult(ros::Duration(duration));
}

void moveOneActuatorToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration,
                        std::vector<double> positions){
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration));
}


/* Publish goal to move_base_simple/goal topic */
void publishSlamGoal(double goal_x, double goal_y, double goal_theta) {
    geometry_msgs::PoseStamped goal_msg;

    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";

    goal_msg.pose.position.x = goal_x;
    goal_msg.pose.position.y = goal_y;
    goal_msg.pose.position.z = 0.0;

    double yaw = radians(goal_theta);
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = sin(yaw / 2.0);
    goal_msg.pose.orientation.w = cos(yaw / 2.0);

    slam_goal_publisher.publish(goal_msg);
    ROS_INFO("Published goal to move_base_simple/goal: (%.2f, %.2f, %.2f)", goal_x, goal_y, goal_theta);
}

/* Publish initial pose to initialpose topic */
void publishSlamInitialPose(double pose_x, double pose_y, double pose_theta) {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.pose.position.x = pose_x;
    pose_msg.pose.pose.position.y = pose_y;
    pose_msg.pose.pose.position.z = 0.0;

    double yaw = radians(pose_theta);
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = sin(yaw / 2.0);
    pose_msg.pose.pose.orientation.w = cos(yaw / 2.0);

    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.068;

    slam_initialpose_publisher.publish(pose_msg);
    ROS_INFO("Published initial pose to initialpose: (%.2f, %.2f, %.2f)", pose_x, pose_y, pose_theta);
}

/*
 * SLAM navigation with action feedback support
 */
int executeSlamNavigationWithFeedback(double goal_x, double goal_y, double goal_theta,
                                      actionlib::SimpleActionServer<cssr_system::setGoalAction>* action_server) {
    if (move_base_client == nullptr || !move_base_client->isServerConnected()) {
        ROS_ERROR("move_base action server is not available. Cannot navigate.");
        return 0;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.position.z = 0.0;

    double yaw = goal_theta;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = sin(yaw / 2.0);
    goal.target_pose.pose.orientation.w = cos(yaw / 2.0);

    ROS_INFO("Sending goal to move_base: (%.2f, %.2f, %.2f degrees)", goal_x, goal_y, degrees(goal_theta));

    move_base_client->sendGoal(goal);

    cssr_system::setGoalFeedback feedback;
    ros::Time last_feedback_time = ros::Time::now();
    double feedback_interval = 1.0 / FEEDBACK_RATE;

    ros::Rate rate(10);
    while (ros::ok()) {
        if (action_server->isPreemptRequested()) {
            ROS_INFO("SLAM navigation preempted");
            move_base_client->cancelGoal();
            return 0;
        }

        actionlib::SimpleClientGoalState state = move_base_client->getState();

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully in SLAM mode!");
            return 1;
        } else if (state == actionlib::SimpleClientGoalState::ABORTED ||
                   state == actionlib::SimpleClientGoalState::REJECTED ||
                   state == actionlib::SimpleClientGoalState::LOST) {
            ROS_WARN("Goal failed with state: %s", state.toString().c_str());
            return 0;
        }

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_feedback_time).toSec() >= feedback_interval) {
            double distance_to_goal = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
            feedback.distance_remaining = distance_to_goal;
            feedback.current_x = current_x;
            feedback.current_y = current_y;
            feedback.current_theta = degrees(current_theta);
            action_server->publishFeedback(feedback);
            last_feedback_time = current_time;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void markWaypointsOnMap(int pathPlanningAlgorithm, Mat mapImage, std::string output_filename, bool debug) {
    drawPathOnMap(valid_waypoints, mapImage, output_filename);
}

void markWaypointsOnConfigMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug) {
    std::string output_filename;
    switch(pathPlanningAlgorithm) {
        case BFS_ALGORITHM:
            output_filename = "configMapWaypointsBFS.png";
            break;
        case DIJKSTRA_ALGORITHM:
            output_filename = "configMapWaypointsDijkstra.png";
            break;
        case ASTAR_ALGORITHM:
            output_filename = "configMapWaypointsAstar.png";
            break;
        case DFS_ALGORITHM:
            output_filename = "configMapWaypointsDFS.png";
            break;
        default:
            output_filename = "configMapWaypoints.png";
    }

    char path_and_input_filename[MAX_FILENAME_LENGTH] = "";
    strcpy(path_and_input_filename, packagedir.c_str());
    strcat(path_and_input_filename, "/robotNavigation/data/");
    strcat(path_and_input_filename, output_filename.c_str());

    drawPathOnMap(valid_waypoints, configurationSpaceImage, path_and_input_filename);
}

void markWaypointsOnEnvironmentMap(int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug) {
    std::string output_filename;
    switch(pathPlanningAlgorithm) {
        case BFS_ALGORITHM:
            output_filename = "environmentMapWaypointsBFS.png";
            break;
        case DIJKSTRA_ALGORITHM:
            output_filename = "environmentMapWaypointsDijkstra.png";
            break;
        case ASTAR_ALGORITHM:
            output_filename = "environmentMapWaypointsAstar.png";
            break;
        case DFS_ALGORITHM:
            output_filename = "environmentMapWaypointsDFS.png";
            break;
        default:
            output_filename = "environmentMapWaypoints.png";
    }

    char path_and_input_filename[MAX_FILENAME_LENGTH] = "";
    strcpy(path_and_input_filename, packagedir.c_str());
    strcat(path_and_input_filename, "/robotNavigation/data/");
    strcat(path_and_input_filename, output_filename.c_str());

    drawPathOnMap(valid_waypoints, mapImage, path_and_input_filename);
}


#ifdef ROS
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
#endif
