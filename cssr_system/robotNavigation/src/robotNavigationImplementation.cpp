/* robotNavigationImplementation.cpp - robot navigation functions implementation.
*
* Author:   Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email:    bgirmash@andrew.cmu.edu
* Date:     June 05, 2025
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

// Publisher for the velocity commands
ros::Publisher navigation_velocity_publisher;
ros::Publisher navigation_pelvis_publisher;

std::atomic<bool> is_moving(false); 

// Size of the map
int x_map_size;
int y_map_size;

int image_width;
int image_height;

double room_width;  // Width of the room in meters
double room_height;  // Height of the room in meters

// Window names to display the maps
string mapWindowName = "Environment Map"; 


locomotionParameterDataType locomotionParameterData;

geometry_msgs::Twist msg;

std::vector<double> leg_home_position = {0.0, 0.0, 0.0};  // Hip pitch, hip roll, knee pitch
std::vector<double> head_home_position = {0.0, 0.0};   // Head pitch and yaw

std::vector<std::vector<int>> graph; 
std::vector<int> robot_path; 
std::vector<pointType> valid_path;
std::vector<waypointType> valid_waypoints;

// 4-way movement (BFS)
int directions_4_way[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// 8-way movement (Dijkstra, A*)
int directions_8_way[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};



/* This defines the callback function for the /robotNavigation/set_goal service */
bool setGoal(cssr_system::setGoal::Request  &service_request, cssr_system::setGoal::Response &service_response){
    // Extract request parameters
    x_goal = service_request.goal_x;
    y_goal = service_request.goal_y;
    theta_goal = service_request.goal_theta;    
    theta_goal = radians(theta_goal); // Convert the angle to radians

    x_goal = roundFloatingPoint(x_goal, 1);
    y_goal = roundFloatingPoint(y_goal, 1);

    int navigation_goal_success = 0;                     

    
    x_start = current_x;
    x_start = roundFloatingPoint(x_start, 1);
    y_start = current_y;
    y_start = roundFloatingPoint(y_start, 1);
    theta_start = current_theta;

    if(x_start > (double) x_map_size/100 || y_start > (double) y_map_size/100){
        ROS_ERROR("Robot pose is outside the map");
        service_response.navigation_goal_success = navigation_goal_success;                
        return true;
    }
    if(x_goal > (double) x_map_size/100 || y_goal > (double) y_map_size/100){
        ROS_ERROR("Goal location is outside the map");
        service_response.navigation_goal_success = navigation_goal_success;              
        return true;
    }

    navigation_goal_success = navigateToGoal(x_start, y_start, theta_start, x_goal, y_goal, theta_goal, pathPlanningAlgorithm, mapImage, configurationSpaceImage, navigation_velocity_publisher, verbose_mode);

    if(navigation_goal_success == 1){
        robotPose[0] = current_x;
        robotPose[1] = current_y;
        robotPose[2] = degrees(current_theta);
        writeRobotPoseInput(robotPose);     
    }

    service_response.navigation_goal_success = navigation_goal_success;               
   
    // Print the response from the service
    ROS_INFO("Response from /robotNavigation/set_goal service: [%ld]\n", (long int)service_response.navigation_goal_success);
    return true;
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

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("Unable to open the data file %s\n", data_path_and_file.c_str());
        promptAndExit(1);
    }

    std::string data_line_read;  
    // Get key-value pairs from the data file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        // Read the x value of the robot pose
        if (param_key == x_key){
            x_value = param_value;
            robot_pose_input[0] = std::stod(param_value);
        }

        // Read the y value of the robot pose
        else if (param_key == y_key){
            y_value = param_value;
            robot_pose_input[1] = std::stod(param_value);
        }

        // Read the theta value of the robot pose
        else if (param_key == theta_key){
            theta_value = param_value;
            robot_pose_input[2] = std::stod(param_value);
        }
    }
    // Close the data file
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

    std::string data_file = "robotPose.dat";    // data filename
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    std::string x_key = "x";                                         // x key
    std::string y_key = "y";                                         // y key
    std::string theta_key = "theta";                                 // theta key

    std::string x_value;                                             // x value
    std::string y_value;                                             // y value
    std::string z_value;                                             // z value
    std::string theta_value;                                         // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open data file for writing
      std::ofstream data_of(data_path_and_file.c_str());
      if (!data_of.is_open()){
         printf("Unable to open the data file %s\n", data_path_and_file.c_str());
         promptAndExit(1);
      }

      // Write the robot pose to the data file
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
    bool debug = false;   // used to turn debug message on
    
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/robotNavigation/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) ROS_INFO("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;   // variable to read the line in the file
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
        if (param_key == key) {                     // if the key is found
            topic_value = param_value;              // set the topic value
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        ROS_ERROR("Unable to find a valid topic for actuator: %s. Please check the topics file.\n", key.c_str());
        return 1;
    }

    *topic_name = topic_value;                      // set the topic name
    return 0;
}

/*  
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */
void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the hand
    ControlClientPtr hand_client;

    // Goal messages for the hand
    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;
    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration)); // Wait for the actuator to reach the specified position

    return;
}


int readConfigurationFile(string* environmentMapFile, string* configurationMapFile, int* path_planning_algorithm, bool* socialDistanceMode, string* robot_topics, string* topics_filename, bool* debug_mode, string* robot_type) {
    std::string config_file = "robotNavigationConfiguration.ini";       // data filename
    std::string config_path;                                            // data path
    std::string config_path_and_file;                                   // data path and filename
     
    std::string environment_map_file_key = "environmentMap";            // camera key
    std::string configuration_map_file_key = "configurationMap";        // realignment threshold key
    std::string path_planning_algorithm_key = "pathPlanning";           // realignment threshold key
    std::string social_distance_mode_key = "socialDistance";            // x offset to head yaw key
    std::string robot_topics_key = "robotTopics";                       // robot topics key
    std::string verbose_mode_key = "verboseMode";                       // verbose mode key
    std::string robot_type_key = "robotType";         

    std::string environment_map_file_value;                             // camera value
    std::string configuration_map_file_value;                           // realignment threshold value
    std::string path_planning_algorithm_value;                            // realignment threshold value
    std::string social_distance_mode_value;                             // x offset to head yaw value
    std::string robot_topics_value;                                     // robot topics value
    std::string verbose_mode_value;                                     // verbose mode value
    std::string robot_type_value;                     

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += "/robotNavigation/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;
    *topics_filename = *robot_topics;
    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        return 1;
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
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
    }
    data_if.close();

    if(*environmentMapFile == "" || *configurationMapFile == "" || *robot_topics == ""){
        printf("Unable to find a valid configuration. Verify you have values in the configuration.\n");
        return 1;
    }
    
    // Since platform is always robot, directly set the topics filename
    *topics_filename = *robot_topics;

    // Verify robot type
    if(*robot_type != "new" && *robot_type != "old") {
        ROS_WARN("Invalid robotType value: %s. Using default 'old'", robot_type_value.c_str());
        *robot_type = "old";
    }

    return 0;
}

/* Print the overt attention configuration */
void printConfiguration(string environmentMapFile, string configurationMapFile, int pathPlanningAlgorithm, bool socialDistanceMode, string robot_topics, string topics_filename, bool debug_mode, string robot_type){
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
}

/******************************************************************************

global variables with the current robot pose

*******************************************************************************/

/* global variables with the initial and current robot pose, the odometry pose, and the difference between the initial pose and the initial odometry pose  */

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

   /* change frame of reference from arbitrary odometry frame of reference to the world frame of reference */

   /* translation of origin */

   x = odom_x + adjustment_x - initial_x;
   y = odom_y + adjustment_y - initial_y;

   /* rotation about origin */

   current_x = x * cos(adjustment_theta) + y * -sin(adjustment_theta);
   current_y = x * sin(adjustment_theta) + y * cos(adjustment_theta);

   current_x += initial_x;
   current_y += initial_y;

   current_theta = odom_theta + adjustment_theta;

   /* check to ensure theta is still in the range -PI to +PI */

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
    // current_theta = fmod(current_theta, 2 * PI);
   
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

   keyword key;   // the key string when reading parameters
   keyword value; // the value string, used for the SHORTEST_PATH_ALGORITHM key
   keyword robot_state; // the robot_state string, used for the ROBOT_AVAILABLE key

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
     printf("Error can't open locomition parameter file %s\n", filename);
     promptAndExit(0);
   }

   /*** set default values ***/

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
   locomotionParameterData->shortest_path_algorithm = ASTAR; // BFS, DIJKSTRA, ASTAR
   locomotionParameterData->robot_available = FALSE; // TRUE/FALSE

   /*** get the key-value pairs ***/

   for (i = 0; i < NUMBER_OF_KEYS; i++)
   {

        if (fgets(input_string, STRING_LENGTH, fp_config) == NULL)
        {
            // Handle error or EOF
            printf("Error: Unable to read all configuration parameters. Expected %d parameters, but could only read %d\n", 
                NUMBER_OF_KEYS, i);
            fclose(fp_config);
            return; 
        }

     /* extract the key */

     sscanf(input_string, " %s", key);

     for (j = 0; j < (int)strlen(key); j++)
        key[j] = tolower(key[j]);

     // if (debug)  printf ("key: %s\n",key);

     for (j = 0; j < NUMBER_OF_KEYS; j++)
     {
        if (strcmp(key, keylist[j]) == 0)
        {
           switch (j)
           {
           case 0:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance)); // position_tolerance
              break;
           case 1:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance_goal)); // angle_tolerance_orienting
              break;
           case 2:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_orienting)); // angle_tolerance_orienting
              break;
           case 3:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_going)); // angle_tolerance_going
              break;
           case 4:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_dq)); // position_gain_dq)
              break;
           case 5:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_dq)); // angle_gain_dq
              break;
           case 6:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_mimo)); // position_gain_mimo
              break;
           case 7:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_mimo)); // angle_gain_mimo
              break;
           case 8:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_linear_velocity)); // min_linear_velocity
              break;
           case 9:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_linear_velocity)); // max_linear_velocity
              break;
           case 10:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_angular_velocity)); // min_angular_velocity
              break;
           case 11:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_angular_velocity)); // max_angular_velocity
              break;
           case 12:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->clearance)); // required clearance between robot and obstacle
              break;
           case 13:
              sscanf(input_string, " %s %s", key, value); // BFS,  Dijkstra,  ASTAR, DFS
              for (k = 0; k < (int)strlen(value); k++)
                 value[k] = tolower(value[k]);
              if (strcmp(value, "bfs") == 0)
                 locomotionParameterData->shortest_path_algorithm = BFS;
              else if (strcmp(value, "dijkstra") == 0)
                 locomotionParameterData->shortest_path_algorithm = DIJKSTRA;
              else if (strcmp(value, "astar") == 0)
                 locomotionParameterData->shortest_path_algorithm = ASTAR;
              else if(strcmp(value, "dfs") == 0)
                locomotionParameterData->shortest_path_algorithm = DFS;
              else
                 locomotionParameterData->shortest_path_algorithm = ASTAR; // default value is A-star
              break;
           case 14:
              sscanf(input_string, " %s %s", key, robot_state); // true/false
              for (k = 0; k < (int)strlen(robot_state); k++)
                 robot_state[k] = tolower(robot_state[k]);
              if (strcmp(robot_state, "true") == 0)
                 locomotionParameterData->robot_available = TRUE;
              else if (strcmp(robot_state, "false") == 0)
                 locomotionParameterData->robot_available = FALSE;
           }
        }
     }
   }

   // printf("value %s\n", value);
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


// Function to build the graph from the image
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
    else { // DIJKSTRA and ASTAR use 8-way movement
        directions = directions_8_way;
        num_directions = 8;
    }

    graph.resize(rows * cols);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (img.at<uchar>(y, x) == 255) {  // Free space
                int current_node = y * cols + x;
                for (int i = 0; i < num_directions; ++i) {
                    int new_x = x + directions[i][0];
                    int new_y = y + directions[i][1];
                    if (new_x >= 0 && new_x < cols && new_y >= 0 && new_y < rows) {
                        if (img.at<uchar>(new_y, new_x) == 255) {  // Neighbor is free space
                            int neighbor_node = new_y * cols + new_x;
                            graph[current_node].push_back(neighbor_node);
                        }
                    }
                }
            }
        }
    }
}


// Manhattan distance heuristic function
int manhattanDistanceHeuristic(int start, int node, int goal, int cols) {
    int goal_x = goal % cols;
    int goal_y = goal / cols;
    int node_x = node % cols;
    int node_y = node / cols;

    int heuristic = 0;
    heuristic = std::abs(goal_x - node_x) + std::abs(goal_y - node_y);

    double D = 6;
    double D2 = D * 1.414;

    int dx = std::abs(node_x - goal_x);
    int dy = std::abs(node_y - goal_y);

    return heuristic;
}

// Euclidean distance heuristic function
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

      int cross_product = abs((dx_start * dy_node )- (dy_start * dx_node));

      heuristic += cross_product * 0.001;
    return heuristic;
}


void convertWorldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y, double room_width, double room_height, int image_width, int image_height) {
    pixel_x = static_cast<int>(world_x * image_width / room_width);
    pixel_y = static_cast<int>(image_height - (world_y * image_height / room_height));
}

void convertPixelToWorld(int pixel_x, int pixel_y, double& world_x, double& world_y, double room_width, double room_height, int image_width, int image_height) {
    world_x = static_cast<double>(pixel_x) * (room_width / image_width);
    world_y = room_height - static_cast<double>(pixel_y) * (room_height / image_height); // Invert Y for world coordinates
}

void drawPathOnMap(const std::vector<waypointType>& valid_waypoints, const cv::Mat& img, const std::string& output_path) {
    cv::Mat output_image;

    cvtColor(img, output_image, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < valid_waypoints.size(); i++) {
        // Draw crosshair
        cv::drawMarker(output_image, cv::Point(valid_waypoints[i].x, valid_waypoints[i].y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    }

  
   for (int i=0; i<valid_path.size(); i++) {
      cv::drawMarker(output_image, cv::Point(valid_path[i].x, valid_path[i].y), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 5, 1);
   }

    cv::imwrite(output_path, output_image); // Save the output image
}

double pathOrientation(std::vector<pointType>& valid_path, int i) {
    double delta_x;
    double delta_y;
    double orientation = 0;

    if (valid_path.size() < 3) {
        orientation = 0;
    } else if (i == 0) {  // Special case: first point
        delta_x = valid_path[i + 1].x - valid_path[i].x;
        delta_y = valid_path[i + 1].y - valid_path[i].y;
    } else if (i == valid_path.size() - 1) {  // Special case: last point
        delta_x = valid_path[i].x - valid_path[i - 1].x;
        delta_y = valid_path[i].y - valid_path[i - 1].y;
    } else {
        delta_x = valid_path[i + 1].x - valid_path[i - 1].x;
        delta_y = valid_path[i + 1].y - valid_path[i - 1].y;
    }

    orientation = std::atan2(-delta_y, delta_x);  // Compute orientation in map frame of reference, not image frame of reference

    return orientation;
}


void computeWaypoints(std::vector<int>& robot_path, std::vector<pointType>& valid_path, std::vector<waypointType>& valid_waypoints, int suppression_factor, double curvature_angle) {
    // Clear previous waypoints
    valid_path.clear();
    valid_waypoints.clear();
    
    if (robot_path.size() < 2) {
        return;
    }
    
    // Convert robot path to valid path (pixel coordinates)
    for (int i = 0; i < robot_path.size(); i++) {
        int x = robot_path[i] % image_width;
        int y = robot_path[i] / image_width;
        valid_path.push_back({x, y});
    }
    
    if (valid_path.size() < 3) {
        // For very short paths, just use start and end
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
    
    // Parameters for intelligent waypoint placement
    const double MIN_TURNING_RADIUS = 30.0;  // Minimum turning radius in pixels
    const double WAYPOINT_SPACING = std::max(40.0, (double)suppression_factor * 2.0);  // Minimum distance between waypoints
    const double DIRECTION_CHANGE_THRESHOLD = curvature_angle * 0.7;  // Threshold for significant direction changes
    const int LOOKAHEAD_DISTANCE = 15;  // How far ahead to look for path analysis
    
    std::vector<waypointType> candidate_waypoints;
    
    // Always add the starting point
    double start_orientation = atan2(valid_path[1].y - valid_path[0].y, valid_path[1].x - valid_path[0].x);
    candidate_waypoints.push_back({valid_path[0].x, valid_path[0].y, start_orientation});
    
    // Analyze path segments for intelligent waypoint placement
    for (int i = LOOKAHEAD_DISTANCE; i < valid_path.size() - LOOKAHEAD_DISTANCE; i++) {
        // Calculate local path curvature and direction changes
        std::vector<double> segment_directions;
        
        // Analyze direction changes in a window around current point
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
        
        // Calculate maximum direction change in this segment
        double max_direction_change = 0.0;
        for (int j = 1; j < segment_directions.size(); j++) {
            double angle_diff = segment_directions[j] - segment_directions[j-1];
            
            // Normalize angle difference to [-PI, PI]
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            
            max_direction_change = std::max(max_direction_change, std::abs(angle_diff));
        }
        
        // Check if this is a significant direction change requiring a waypoint
        bool significant_turn = max_direction_change > DIRECTION_CHANGE_THRESHOLD;
        
        // Check if we're far enough from the last waypoint
        bool sufficient_distance = true;
        if (!candidate_waypoints.empty()) {
            double dist = sqrt(pow(valid_path[i].x - candidate_waypoints.back().x, 2) + 
                             pow(valid_path[i].y - candidate_waypoints.back().y, 2));
            sufficient_distance = (dist >= WAYPOINT_SPACING);
        }
        
        // Calculate path complexity ahead (used for velocity planning)
        double path_complexity = 0.0;
        for (int j = i; j < std::min(i + LOOKAHEAD_DISTANCE*2, (int)valid_path.size()-1); j++) {
            if (j < segment_directions.size()-1) {
                path_complexity += std::abs(segment_directions[j+1] - segment_directions[j]);
            }
        }
        
        // Place waypoint if it meets criteria
        if ((significant_turn && sufficient_distance) || 
            (sufficient_distance && path_complexity > DIRECTION_CHANGE_THRESHOLD * 2)) {
            
            // Calculate optimal approach angle for this waypoint
            double approach_angle = calculateOptimalApproachAngle(valid_path, i, LOOKAHEAD_DISTANCE);
            candidate_waypoints.push_back({valid_path[i].x, valid_path[i].y, approach_angle});
        }
    }
    
    // Always add the final destination
    int last_idx = valid_path.size() - 1;
    double end_orientation = (valid_path.size() > 1) ? 
        atan2(valid_path[last_idx].y - valid_path[last_idx-1].y, 
              valid_path[last_idx].x - valid_path[last_idx-1].x) : 0.0;
    candidate_waypoints.push_back({valid_path[last_idx].x, valid_path[last_idx].y, end_orientation});
    
    // Optimize waypoint sequence for smooth transitions
    valid_waypoints = optimizeWaypointSequence(candidate_waypoints, MIN_TURNING_RADIUS);
    
    // Ensure minimum waypoint count for proper navigation
    if (valid_waypoints.size() < 2 && valid_path.size() >= 2) {
        valid_waypoints.clear();
        valid_waypoints.push_back({valid_path[0].x, valid_path[0].y, start_orientation});
        valid_waypoints.push_back({valid_path.back().x, valid_path.back().y, end_orientation});
    }
}

// Helper function to calculate optimal approach angle for smooth transitions
double calculateOptimalApproachAngle(const std::vector<pointType>& path, int current_idx, int lookahead) {
    if (current_idx >= path.size()) return 0.0;
    
    // Look ahead to determine the best approach angle
    int future_idx = std::min(current_idx + lookahead, (int)path.size() - 1);
    int past_idx = std::max(current_idx - lookahead/2, 0);
    
    // Calculate weighted average direction considering both past and future path
    double future_weight = 0.7;  // Prioritize future direction
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
    
    // Weighted combination for smooth transitions
    double optimal_angle = future_weight * future_angle + past_weight * past_angle;
    
    return optimal_angle;
}

// Helper function to optimize waypoint sequence for turning radius constraints
std::vector<waypointType> optimizeWaypointSequence(const std::vector<waypointType>& candidates, double min_turning_radius) {
    if (candidates.size() <= 2) return candidates;
    
    std::vector<waypointType> optimized;
    optimized.push_back(candidates[0]);  // Always keep start
    
    for (int i = 1; i < candidates.size() - 1; i++) {
        const auto& prev = optimized.back();
        const auto& current = candidates[i];
        const auto& next = candidates[i + 1];
        
        // Calculate required turning radius at this waypoint
        double dist_to_prev = sqrt(pow(current.x - prev.x, 2) + pow(current.y - prev.y, 2));
        double dist_to_next = sqrt(pow(next.x - current.x, 2) + pow(next.y - current.y, 2));
        
        // Calculate angle between incoming and outgoing vectors
        double angle_in = atan2(current.y - prev.y, current.x - prev.x);
        double angle_out = atan2(next.y - current.y, next.x - current.x);
        double turn_angle = angle_out - angle_in;
        
        // Normalize turn angle
        while (turn_angle > M_PI) turn_angle -= 2 * M_PI;
        while (turn_angle < -M_PI) turn_angle += 2 * M_PI;
        
        // Check if this waypoint creates a feasible turn
        if (std::abs(turn_angle) > 0.1 && dist_to_prev > min_turning_radius * 0.5) {
            // Adjust waypoint angle for smooth transition
            waypointType adjusted = current;
            adjusted.theta = (angle_in + angle_out) / 2.0;  // Average of incoming and outgoing angles
            optimized.push_back(adjusted);
        }
    }
    
    optimized.push_back(candidates.back());  // Always keep end
    return optimized;
}


//##########################################################################################################

void printWaypoints(std::vector<waypointType>& valid_waypoints, double room_width, double room_height, int image_width, int image_height){
    for(int i = 0; i < valid_waypoints.size(); i++){
        // Convert to world coordinates
        double world_x;
        double world_y;
        convertPixelToWorld(valid_waypoints[i].x, valid_waypoints[i].y, world_x, world_y, room_width, room_height, image_width, image_height);

        // ROS_INFO("Path Point: (%.2f, %.2f)", world_x, world_y);
        ROS_INFO("Waypoint %d: x = %.2f, y = %.2f, theta = %.2f\n", i, world_x, world_y, valid_waypoints[i].theta);
    }
}

// A* pathfinding algorithm
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
                int priority = new_dist + heuristic(current, neighbor, goal, cols);
                pq.push({priority, neighbor});
            }
        }
    }

    return {};
}



std::vector<int> bfs(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::queue<int> queue;
    std::vector<int> came_from(graph.size(), -1);
    std::vector<bool> visited(graph.size(), false);

    queue.push(start);
    visited[start] = true;

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

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
                visited[neighbor] = true;
                came_from[neighbor] = current;
                queue.push(neighbor);
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
    std::stack<int> stack;
    std::vector<int> came_from(graph.size(), -1);
    std::vector<bool> visited(graph.size(), false);

    stack.push(start);
    visited[start] = true;

    while (!stack.empty()) {
        int current = stack.top();
        stack.pop();

        if (current == goal) {
            // Reconstruct path
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Get neighbors and sort by distance to goal (CLOSEST FIRST)
        std::vector<int> neighbors = graph[current];
        
        // Sort neighbors by Manhattan distance to goal
        std::sort(neighbors.begin(), neighbors.end(), [goal](int a, int b) {
            int cols = image_width; // Your global variable
            
            int ax = a % cols, ay = a / cols;
            int bx = b % cols, by = b / cols;
            int gx = goal % cols, gy = goal / cols;
            
            int dist_a = abs(ax - gx) + abs(ay - gy);
            int dist_b = abs(bx - gx) + abs(by - gy);
            
            return dist_a < dist_b; // Closest to goal first
        });

        // Add neighbors to stack in REVERSE order (so closest gets processed first)
        for (auto it = neighbors.rbegin(); it != neighbors.rend(); ++it) {
            int neighbor = *it;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                came_from[neighbor] = current;
                stack.push(neighbor);
            }
        }
    }
    
    return {};
}
/***************************************************************************************************************************

   General purpose function definitions 

****************************************************************************************************************************/

/* return the sign of a number as +/- 1 */

int signnum(double x)
{
    if (x >= 0.0){
        return 1;
    }
    return -1;
}

void displayErrorAndExit(char error_message[])
{
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void printMessageToFile(FILE *fp, char message[])
{
   fprintf(fp, "The message is: %s\n", message);
}

#ifdef ROS
/**
 Linux (POSIX) implementation of kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

std::vector<cv::Point> calculateBezierCurve(const std::vector<cv::Point>& points, int numPoints) {
    std::vector<cv::Point> curvePoints;
    int n = points.size() - 1;
    
    for (double t = 0; t <= 1; t += 1.0 / numPoints) {
        double x = 0, y = 0;
        for (int i = 0; i <= n; i++) {
            double coefficient = std::pow(1 - t, n - i) * std::pow(t, i) * 
                                 std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
            x += coefficient * points[i].x;
            y += coefficient * points[i].y;
        }
        curvePoints.push_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
    }
    
    return curvePoints;
}

int planRobotPath(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, bool debug){
   // Vertex number of the start and goal cells
    int robot;                              // vertex number of start cell
    int goal;                               // vertex number of goal cell

    // Map values of the start location
    int x_start_map;
    int y_start_map;

    // Map values of the goal location
    int x_goal_map;
    int y_goal_map;

    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    // Waypoint values
    double x_waypoint;
    double y_waypoint;
    double theta_waypoint;

    // Convert the start location to map coordinates
    x_start_map = y_map_size - (int) (start_y * 100);  // NB: convert to cm and change frame of reference
    y_start_map = (int) (start_x * 100);               // ibid.

    convertWorldToPixel(start_x, start_y, start_pixel_x, start_pixel_y, room_width, room_height, image_width, image_height);
    convertWorldToPixel(goal_x, goal_y, end_pixel_x, end_pixel_y, room_width, room_height, image_width, image_height);


    // Convert the goal location to map coordinates
    x_goal_map = y_map_size - (int) (goal_y * 100);   // NB: convert to cm and change frame of reference
    y_goal_map = (int) (goal_x * 100);                

    robot = start_pixel_y * image_width + start_pixel_x;
    goal = end_pixel_y * image_width + end_pixel_x;


    int heuristic_choice = MANHATTAN_HEURISTIC;  // Choose heuristic for A* (Manhattan or Euclidean)
    
    robot_path.clear();
    valid_path.clear();
    valid_waypoints.clear();
   
    if (pathPlanningAlgorithm == BFS) {
        robot_path = bfs(robot, goal, graph);
    } else if (pathPlanningAlgorithm == DIJKSTRA) {
        robot_path = dijkstra(robot, goal, graph);
    } else if (pathPlanningAlgorithm == ASTAR) {
        robot_path = astar(robot, goal, graph, image_width, heuristic_choice);
    } else if (pathPlanningAlgorithm == DFS) {
        robot_path = dfs(robot, goal, graph);
    }

    /* get the waypoints  */
    /* ------------------ */

    if(robot_path.empty()){
        ROS_ERROR("No path found from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", start_x, start_y, start_theta, goal_x, goal_y, goal_theta);
        return 0;
    }  
    computeWaypoints(robot_path, valid_path, valid_waypoints, SUPPRESSION_FACTOR, CURVATURE_THRESHOLD);

   // markWaypointsOnConfigMap(pathPlanningAlgorithm, mapImage, configurationSpaceImage, debug);
   std::string output_filename = "environmentMapWaypoints";
   markWaypointsOnMap(pathPlanningAlgorithm, mapImage, output_filename, debug);

   output_filename = "configMapWaypoints";
   markWaypointsOnMap(pathPlanningAlgorithm, configurationSpaceImage, output_filename, debug);

   return 1;              // Return 1 if the path is found
}

void markWaypointsOnMap(int pathPlanningAlgorithm, Mat mapImage, std::string output_filename, bool debug){
   // parameters for image write
   vector<int> compressionParams;  

   // Map image in colour
   Mat mapImageColor;

   // Map image in large format
   Mat mapImageLarge;

   // scale the map and configuration images by this factor before displaying
   double image_display_scale_factor  = 4.0;

   /* Draw path in colour with waypoint and display map and configuration space */
   /* ------------------------------------------------------------------------- */
   
   cvtColor(mapImage,mapImageColor,COLOR_GRAY2BGR);

   /* Draw a grid on the output map imitating the size of a tile in the laboratory. This can be commented out */
   int dist=60;

   for(int i=0;i<mapImageColor.rows;i+=dist)
      line(mapImageColor,Point(0,i),Point(mapImageColor.cols,i),Scalar(0,0,0));

   for(int i=0;i<mapImageColor.cols;i+=dist)
      line(mapImageColor,Point(i,0),Point(i,mapImageColor.rows),Scalar(0,0,0));

   /* Mark the waypoints on the map */

   for (int i = 0; i < valid_waypoints.size(); i++) {
        // Draw crosshair
        cv::drawMarker(mapImageColor, cv::Point(valid_waypoints[i].x, valid_waypoints[i].y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    }

   for (int i=0; i<valid_path.size(); i++) {
      cv::drawMarker(mapImageColor, cv::Point(valid_path[i].x, valid_path[i].y), cv::Scalar(0, 0, 255), cv::MARKER_SQUARE, 1, 1);
   }

   resize(mapImageColor, mapImageLarge, Size(mapImage.cols * image_display_scale_factor, mapImage.rows * image_display_scale_factor), INTER_NEAREST);

   compressionParams.push_back(IMWRITE_PNG_COMPRESSION);
   compressionParams.push_back(9);                                  // 9 implies maximum compression   

   std::string name_extension = "Astar";

   if(pathPlanningAlgorithm == BFS_ALGORITHM){
      name_extension = "BFS";

   }else if(pathPlanningAlgorithm == DIJKSTRA_ALGORITHM){
      name_extension = "Dijkstra";
   }
   else if(pathPlanningAlgorithm == ASTAR_ALGORITHM){
      name_extension = "Astar";
   }
   else if(pathPlanningAlgorithm == DFS_ALGORITHM){
      name_extension = "DFS";
   }

   std::string filename = output_filename + name_extension + ".png";

   saveWaypointMap(compressionParams, mapImageLarge, filename); // Save the map with the path and waypoints
}

int navigateToGoal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int pathPlanningAlgorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug){
   // Set the publish rate for the velocity commands
   ros::Rate rate(PUBLISH_RATE); // Publish  at this rate (in Hz)  until the node is shut down


   int path_found = 0;
   int robot_moved = 0;
   path_found = planRobotPath(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, pathPlanningAlgorithm, mapImage, configurationSpaceImage, debug);
   if (path_found){
      if(debug){
         printf("Path found from (%5.3f %5.3f %5.3f) to (%5.3f %5.3f %5.3f)\n", start_x, start_y, degrees(start_theta), goal_x, goal_y, degrees(goal_theta));
         // printWaypoints(valid_waypoints, room_width, room_height, image_width, image_height);
      }
      robot_moved = moveRobot(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, velocity_publisher, rate, debug);
   }
   return robot_moved;
}

ControlClientPtr createClient(const std::string& topic_name) {
    // Create a new action client
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 10;                                            // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;                                        // return the action client if the server is available
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    // throw std::runtime_error("Error creating action client for " + topic_name + " controller: Server not available");
    return nullptr;                                                     // return nullptr if the server is not available
}


int goToHome(std::string actuator, std::string topics_filename, bool debug){
    // ros::Duration(0.5).sleep(); // Wait for one second to ensure that the joint states are updated
    std::vector<double> actuator_state;                             // stores the current state of the actuator joints
    std::vector<double> actuator_home_position;                     // stores the home position of the actuator joints
    ControlClientPtr actuator_client;                               // action client to control the actuator joints
    std::vector<std::string> actuator_joint_names;                  // stores the joint names of the actuator joints
    std::string actuator_topic;                                     // stores the topic of the actuator for control
    int number_of_joints;                                           // stores the number of joints of the actuator
    double home_duration;                                           // stores the duration to move to the home position

    // Set the open hand flag and the hand topic. ust for default, the hand is not to be open in home position
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Extract the actuator topic
    if(extractTopic(actuator, topics_filename, &actuator_topic)){
        return 0;
    }

    // Set the home duration
    home_duration = 0.5;

    if(actuator == "Leg"){                         // Leg
      //   actuator_state = leg_joint_states;
        actuator_home_position = leg_home_position;
        actuator_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
        actuator_client = createClient(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }
    else if(actuator == "Head"){                        // Head
      //   actuator_state = head_joint_states;
        actuator_home_position = head_home_position;
        actuator_joint_names = {"HeadPitch", "HeadYaw"};
        actuator_client = createClient(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }

    // Vectors to store the trajectory information (positions, velocities, accelerations and durations)
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

   //  // Compute the trajectory to get to the home position
   //  compute_trajectory(actuator_state, actuator_home_position, actuator_state.size(), home_duration, positions_t, velocities_t, accelerations_t, duration_t);

    // Move the joints of the actuator depending on the interpolation means selected                                                  // Linear interpolation
   moveToPosition(actuator_client, actuator_joint_names, home_duration, open_hand, hand, hand_topic, "home", actuator_home_position);

    return 1;


}


void setOdometryPose(double x, double y, double theta)
{

   bool debug = false;

   sleep(1); // allow time for messages to be published on the odom topic
   ros::spinOnce();

   /* store the initial pose */
   initial_x = x;
   initial_y = y;
   initial_theta = theta;

   /* calculate the adjustment to the pose, i.e. the difference between the initial pose and odometry pose */
   adjustment_x = x - odom_x;
   adjustment_y = y - odom_y;
   adjustment_theta = theta - odom_theta;


   if (debug)
   {
      printf("setOdometryPose: odom_x,y,theta %.2f %.2f %.2f  adjustment_x, y, theta %.2f %.2f %.2f\n", odom_x, odom_y, odom_theta,
             adjustment_x, adjustment_y, adjustment_theta);
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
    
    // Enhanced multi-waypoint navigation with smooth flow
    return executeSmoothWaypointNavigation(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, 
                                         velocity_publisher, rate, debug);
}

// New function for smooth waypoint navigation
int executeSmoothWaypointNavigation(double start_x, double start_y, double start_theta, 
                                   double goal_x, double goal_y, double goal_theta,
                                   ros::Publisher velocity_publisher, ros::Rate rate, bool debug) {
    
    geometry_msgs::Twist msg;
    
    // Navigation parameters
    const double WAYPOINT_APPROACH_RADIUS = 0.25;  // Distance to start transitioning to next waypoint
    const double WAYPOINT_CAPTURE_RADIUS = 0.15;   // Distance to consider waypoint reached
    const double LOOKAHEAD_DISTANCE = 0.4;         // How far ahead to look for path planning
    const double MAX_LINEAR_VEL = locomotionParameterData.max_linear_velocity * 0.9;
    const double MAX_ANGULAR_VEL = locomotionParameterData.max_angular_velocity * 0.9;
    const double MIN_LINEAR_VEL = locomotionParameterData.min_linear_velocity;
    const double MIN_ANGULAR_VEL = locomotionParameterData.min_angular_velocity;
    
    // State variables
    int current_waypoint_idx = 0;
    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    bool navigation_complete = false;
    
    // Conversion helper for waypoints
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
        ros::spinOnce();  // Update current pose
        
        // Get current target waypoint
        auto [target_x, target_y, target_theta] = getWaypointWorldCoords(current_waypoint_idx);
        
        // Calculate distance to current waypoint
        double distance_to_waypoint = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        
        // Check if we should advance to next waypoint
        if (distance_to_waypoint < WAYPOINT_CAPTURE_RADIUS) {
            current_waypoint_idx++;
            if (current_waypoint_idx >= valid_waypoints.size()) {
                // Navigate to final goal with precise positioning
                navigation_complete = true;
                break;
            }
            
            if (debug) {
                printf("Advanced to waypoint %d\n", current_waypoint_idx);
            }
            continue;
        }
        
        // Predictive navigation: look ahead to next waypoint for smooth transitions
        double lookahead_x = target_x;
        double lookahead_y = target_y;
        double lookahead_weight = 0.0;
        
        if (current_waypoint_idx + 1 < valid_waypoints.size() && distance_to_waypoint < WAYPOINT_APPROACH_RADIUS) {
            auto [next_x, next_y, next_theta] = getWaypointWorldCoords(current_waypoint_idx + 1);
            
            // Calculate blend factor based on distance to current waypoint
            lookahead_weight = 1.0 - (distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            lookahead_weight = std::min(0.6, lookahead_weight);  // Limit influence
            
            // Blend current and next waypoint for smooth curve
            lookahead_x = target_x * (1.0 - lookahead_weight) + next_x * lookahead_weight;
            lookahead_y = target_y * (1.0 - lookahead_weight) + next_y * lookahead_weight;
            
            if (debug && lookahead_weight > 0.1) {
                printf("Blending waypoints: weight=%.2f, target=(%.2f,%.2f), lookahead=(%.2f,%.2f)\n", 
                       lookahead_weight, target_x, target_y, lookahead_x, lookahead_y);
            }
        }
        
        // Calculate navigation vectors
        double goal_direction = atan2(lookahead_y - current_y, lookahead_x - current_x);
        double angle_error = goal_direction - current_theta;
        
        // Normalize angle error
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        
        // Velocity calculation with smooth acceleration/deceleration
        double target_linear_vel = MAX_LINEAR_VEL;
        
        // Reduce speed for tight turns
        double turn_factor = 1.0 - std::min(1.0, std::abs(angle_error) / (M_PI/3));
        target_linear_vel *= (0.4 + 0.6 * turn_factor);  // Never go below 40% speed
        
        // Reduce speed when approaching waypoints (except when blending)
        if (lookahead_weight < 0.3) {  // Not in blending mode
            double approach_factor = std::min(1.0, distance_to_waypoint / WAYPOINT_APPROACH_RADIUS);
            target_linear_vel *= (0.5 + 0.5 * approach_factor);  // Minimum 50% speed
        }
        
        target_linear_vel = std::max(target_linear_vel, MIN_LINEAR_VEL);
        
        // Angular velocity calculation
        double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error;
        target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);
        
        if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > 0.05) {
            target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
        }
        
        // Smooth acceleration limits
        double accel_limit = 0.15;
        double angular_accel_limit = 0.2;
        
        // Apply acceleration limits for smooth motion
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
        
        // Publish velocity commands
        msg.linear.x = current_linear_velocity;
        msg.angular.z = current_angular_velocity;
        velocity_publisher.publish(msg);
        
        if (debug) {
            printf("WP %d: dist=%.2f, angle_err=%.2f, lin_vel=%.2f, ang_vel=%.2f, blend=%.2f\n", 
                   current_waypoint_idx, distance_to_waypoint, angle_error, 
                   current_linear_velocity, current_angular_velocity, lookahead_weight);
        }
        
        rate.sleep();
    }
    
    // Final precise navigation to goal
    if (navigation_complete) {
        if (debug) {
            printf("Executing final precision navigation to goal\n");
        }
        
        // Use the improved single-point navigation for final approach
        goToPoseDQ(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate);
    }
    
    // Ensure complete stop
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_publisher.publish(msg);
    
    if (debug) {
        printf("Smooth waypoint navigation completed successfully\n");
    }
    
    return 1;
}


void saveWaypointMap(vector<int> compressionParams, Mat mapImageLarge, string fileName){
    char path_and_input_filename[MAX_FILENAME_LENGTH]        = "";
    std::string navigation_pathway_filename = fileName;  // name of the output pathway image

    /* Combine the name for the output pathway image*/
    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, navigation_pathway_filename.c_str());

    cv::imwrite(path_and_input_filename, mapImageLarge, compressionParams);   // write the image to a file
    printf("Navigation pathway image is saved in %s\n", path_and_input_filename);
    //imshow(mapWindowName, mapImageLarge); // Display map image
}


void goToPoseDQ(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate) {
    // This is now a wrapper that calls the new smooth locomotion system
    // with single-waypoint mode for backward compatibility
    
    bool debug = false;
    geometry_msgs::Twist msg;
    
    double goal_x = x;
    double goal_y = y;
    double goal_theta = theta;
    
    // Enhanced smooth motion parameters
    const double APPROACH_DISTANCE_THRESHOLD = 0.15;  // Distance to start slowing down
    const double POSITION_TOLERANCE = locomotionParameterData.position_tolerance_goal;
    const double ANGLE_TOLERANCE = locomotionParameterData.angle_tolerance_orienting;
    const double MAX_LINEAR_VEL = locomotionParameterData.max_linear_velocity * 0.8;  // Slightly reduced for smoothness
    const double MAX_ANGULAR_VEL = locomotionParameterData.max_angular_velocity * 0.8;
    const double MIN_LINEAR_VEL = locomotionParameterData.min_linear_velocity;
    const double MIN_ANGULAR_VEL = locomotionParameterData.min_angular_velocity;
    
    // State variables for smooth motion
    double current_linear_velocity = 0.0;
    double current_angular_velocity = 0.0;
    double position_error = 0.0;
    double angle_error = 0.0;
    
    // Motion phases
    enum MotionPhase { APPROACHING, FINE_POSITIONING, FINAL_ORIENTATION, COMPLETED };
    MotionPhase phase = APPROACHING;
    
    while (phase != COMPLETED && ros::ok()) {
        ros::spinOnce();  // Update current pose
        
        // Calculate current errors
        position_error = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));
        
        if (phase == APPROACHING || phase == FINE_POSITIONING) {
            // Calculate direction to goal
            double goal_direction = atan2(goal_y - current_y, goal_x - current_x);
            angle_error = goal_direction - current_theta;
            
            // Normalize angle error
            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;
            
            // Check if we should transition to final orientation
            if (position_error <= POSITION_TOLERANCE) {
                phase = FINAL_ORIENTATION;
                continue;
            }
            
            // Smooth velocity control with distance-based scaling
            double distance_scale = std::min(1.0, position_error / APPROACH_DISTANCE_THRESHOLD);
            double angular_scale = 1.0 - std::min(1.0, std::abs(angle_error) / (M_PI/4));  // Reduce linear velocity for large turns
            
            // Calculate target velocities
            double target_linear_vel = MAX_LINEAR_VEL * distance_scale * angular_scale;
            target_linear_vel = std::max(target_linear_vel, MIN_LINEAR_VEL);
            
            double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error;
            target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);
            if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > 0.05) {
                target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
            }
            
            // Smooth acceleration/deceleration
            double linear_accel = 0.1;  // Acceleration factor
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
            
            if (debug) {
                printf("Smooth approach - Pos error: %.3f, Angle error: %.3f, Lin vel: %.3f, Ang vel: %.3f\n", 
                       position_error, angle_error, current_linear_velocity, current_angular_velocity);
            }
            
        } else if (phase == FINAL_ORIENTATION) {
            // Final orientation adjustment
            angle_error = theta - current_theta;
            
            // Normalize angle error
            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;
            
            if (std::abs(angle_error) <= ANGLE_TOLERANCE) {
                phase = COMPLETED;
                continue;
            }
            
            // Stop linear motion, smooth angular adjustment
            current_linear_velocity *= 0.9;  // Gradual stop
            if (current_linear_velocity < 0.01) current_linear_velocity = 0.0;
            
            double target_angular_vel = locomotionParameterData.angle_gain_dq * angle_error * 0.8;  // Slightly gentler
            target_angular_vel = std::max(std::min(target_angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);
            
            if (std::abs(target_angular_vel) < MIN_ANGULAR_VEL && std::abs(angle_error) > ANGLE_TOLERANCE) {
                target_angular_vel = MIN_ANGULAR_VEL * ((target_angular_vel >= 0) ? 1 : -1);
            }
            
            // Smooth angular deceleration as we approach target
            if (std::abs(angle_error) < 0.2) {  // Within ~11 degrees
                target_angular_vel *= (std::abs(angle_error) / 0.2);
            }
            
            current_angular_velocity = target_angular_vel;
            
            msg.linear.x = current_linear_velocity;
            msg.angular.z = current_angular_velocity;
            
            if (debug) {
                printf("Final orientation - Angle error: %.3f, Ang vel: %.3f\n", 
                       angle_error, current_angular_velocity);
            }
        }
        
        pub.publish(msg);
        rate.sleep();
    }
    
    // Ensure complete stop
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
    
    if (debug) {
        printf("Smooth locomotion completed - Final pos error: %.3f, Final angle error: %.3f\n", 
               position_error, angle_error);
    }
}



void goToPoseDQFunctional(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate)
{

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;
   double current_linear_velocity = 0;

   int number_of_ramp_up_steps = 20;

   int mode; // GOING or ORIENTING

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   mode = ORIENTING; // divide and conquer always starts by adjusing the heading

   do
   {

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback and publish the pose on the odom topic

      position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                            (goal_y - current_y) * (goal_y - current_y));

      goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
      // printf("Current theta %.3f\n", current_theta);
      angle_error = goal_direction - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,             */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radians or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                  */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      // if (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) {
      if (((mode == ORIENTING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)) || // low angular tolerance when orienting to get the best initial heading
          ((mode == GOING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_going)))
      { // high angular tolerance when going so we don't have to correct the heading too often

         /* if the robot is not oriented correctly, adjust the heading */

         if (debug)
            printf("Orienting\n");

         mode = ORIENTING; // reset mode from GOING to ORIENTING to ensure we use the lower angular tolerance when reorienting

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         msg.linear.x = 0;

         angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
            msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
            msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
            msg.angular.z = angular_velocity;
      }
      else if (position_error > locomotionParameterData.position_tolerance)
      {

         mode = GOING;

         /* if the robot has not reached the goal, adjust the distance */

         if (debug)
            printf("Going\n");

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         linear_velocity = locomotionParameterData.position_gain_dq * position_error;

         if (linear_velocity < locomotionParameterData.min_linear_velocity)
            linear_velocity = locomotionParameterData.min_linear_velocity;
         else if (linear_velocity > locomotionParameterData.max_linear_velocity)
            linear_velocity = locomotionParameterData.max_linear_velocity;

         /* if stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

         if (current_linear_velocity == 0)
         {

            for (int i = 1; i < number_of_ramp_up_steps; i++)
            {
                msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
                msg.angular.z = 0;

                if (debug)
                {
         printf("Ramping up velocity\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n", msg.linear.x, msg.angular.z);
                }

                pub.publish(msg); // Publish the message

                rate.sleep(); // Wait until it's time for another iteration
            }
            current_linear_velocity = linear_velocity;
         }

         msg.linear.x = linear_velocity;
         msg.angular.z = 0;
      }

      if (debug)
      {
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

   } while ((position_error > locomotionParameterData.position_tolerance_goal) && ros::ok());

   /* the robot has reached the destination so             */
   /* adjust the orientation to match the goal orientation */

   do
   {

      if (debug)
         printf("Orienting\n");

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback

      angle_error = goal_theta - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      msg.linear.x = 0;

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */

      angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
         msg.angular.z = angular_velocity;

      if (debug)
      {
         printf("Orienting\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      if((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)){

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration
      }

   } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

   msg.linear.x = 0;
   msg.angular.z = 0;

   pub.publish(msg); // Publish the message
   // rate.sleep(); // Wait until it's time for another iteration


}


/******************************************************************************

goToPoseDQ

Use the divide and conquer algorithm to drive the robot to a given pose

*******************************************************************************/

void goToPoseDQOld(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate)
{

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;
   double current_linear_velocity = 0;

   int number_of_ramp_up_steps = 20;

   int mode; // GOING or ORIENTING

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   mode = ORIENTING; // divide and conquer always starts by adjusing the heading

   do
   {

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback and publish the pose on the odom topic

      position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                            (goal_y - current_y) * (goal_y - current_y));

      goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
      // printf("Current theta %.3f\n", current_theta);
      angle_error = goal_direction - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,             */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radians or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                  */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      // if (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) {
      if (((mode == ORIENTING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)) || // low angular tolerance when orienting to get the best initial heading
          ((mode == GOING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_going)))
      { // high angular tolerance when going so we don't have to correct the heading too often

         /* if the robot is not oriented correctly, adjust the heading */

         if (debug)
            printf("Orienting\n");

         mode = ORIENTING; // reset mode from GOING to ORIENTING to ensure we use the lower angular tolerance when reorienting

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         msg.linear.x = 0;

         angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
            msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
            msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
            msg.angular.z = angular_velocity;
      }
      else if (position_error > locomotionParameterData.position_tolerance)
      {

         mode = GOING;

         /* if the robot has not reached the goal, adjust the distance */

         if (debug)
            printf("Going\n");

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         linear_velocity = locomotionParameterData.position_gain_dq * position_error;

         if (linear_velocity < locomotionParameterData.min_linear_velocity)
            linear_velocity = locomotionParameterData.min_linear_velocity;
         else if (linear_velocity > locomotionParameterData.max_linear_velocity)
            linear_velocity = locomotionParameterData.max_linear_velocity;

         /* if stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

         if (current_linear_velocity == 0)
         {

            for (int i = 1; i < number_of_ramp_up_steps; i++)
            {
                msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
                msg.angular.z = 0;

                if (debug)
                {
         printf("Ramping up velocity\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n", msg.linear.x, msg.angular.z);
                }

                pub.publish(msg); // Publish the message

                rate.sleep(); // Wait until it's time for another iteration
            }
            current_linear_velocity = linear_velocity;
         }

         msg.linear.x = linear_velocity;
         msg.angular.z = 0;
      }

      if (debug)
      {
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

   } while ((position_error > locomotionParameterData.position_tolerance) && ros::ok());

   /* the robot has reached the destination so             */
   /* adjust the orientation to match the goal orientation */

   do
   {

      if (debug)
         printf("Orienting\n");

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback

      angle_error = goal_theta - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      msg.linear.x = 0;

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */

      angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
         msg.angular.z = angular_velocity;

      if (debug)
      {
         printf("Orienting\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      if((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)){

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration
      }

   } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

   msg.linear.x = 0;
   msg.angular.z = 0;

   pub.publish(msg); // Publish the message
   rate.sleep(); // Wait until it's time for another iteration


}


void stabilizeWaist()
{
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("HipRoll");
    msg.joint_angles.push_back(0.1);  // Set hip pitch to 0 (upright position)
    msg.joint_names.push_back("HipPitch");
    msg.joint_angles.push_back(-0.1);  // Set hip roll to 0 (upright position)
    msg.joint_names.push_back("KneePitch");
    msg.joint_angles.push_back(0.0);  // Set knee pitch to 0 (upright position)
    msg.speed = 0.2;  // Adjust speed as needed
    msg.relative = 0;  // Absolute position

    navigation_pelvis_publisher.publish(msg);

}

void stabilizeWaistContinuously()
{
    ros::Rate rate(10);  // Adjust the rate as needed
    while (ros::ok())
    {
      //   if (is_moving)
        {
            stabilizeWaist();
        }
        rate.sleep();
    }
}

/**********************************************************************************************************************

goToPoseMIMO

Use the MIMO algorithm to drive the robot to a given position and then adjust orientation to achieve the required  pose

***********************************************************************************************************************/

 void goToPoseMIMONew(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
 {

   bool debug = false;

   geometry_msgs::Twist msg;

    naoqi_bridge_msgs::JointAnglesWithSpeed pelvis_msg;
   //  pelvis_msg.joint_names.push_back("HipRoll");
   //  pelvis_msg.joint_angles.push_back(0.0);  // Set hip pitch to 0 (upright position)
    pelvis_msg.joint_names.push_back("HipPitch");
    pelvis_msg.joint_angles.push_back(0.0);  // Set hip roll to 0 (upright position)
   //  pelvis_msg.joint_names.push_back("KneePitch");
   //  pelvis_msg.joint_angles.push_back(0.0);  // Set knee pitch to 0 (upright position)
    pelvis_msg.speed = 1.0;  // Adjust speed as needed
    pelvis_msg.relative = 0;  // Absolute position


   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;

   static double current_linear_velocity = 0; // make this static so that it's valid on the next call

   int number_of_ramp_up_steps = 20;

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   do
   {

     /* get the current pose */

     ros::spinOnce(); // Let ROS take over to handle the callback

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
     /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
     /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     /* set linear and angular velocities, taking care not to use values that exceed maximum values */
     /* or use values that are less than minimum values needed to produce a response in the robot   */

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     /* don't slow down if driving to a waypoint */

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

     /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

     if (current_linear_velocity == 0)
     {

         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
      msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
      msg.angular.z = 0;

      if (debug)
      {
              printf("Ramping up velocity\n");
              // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
              // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
              printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
              printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      navigation_pelvis_publisher.publish(pelvis_msg);
      pub.publish(msg); // Publish the message
      is_moving = true;
      // navigation_pelvis_publisher.publish(trajectory_msg);

      rate.sleep(); // Wait until it's time for another iteration
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;

     if (debug)
     {
         printf("Going\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }

      is_moving = true;
      navigation_pelvis_publisher.publish(pelvis_msg);
     pub.publish(msg); // Publish the message
   //   navigation_pelvis_publisher.publish(trajectory_msg);

     rate.sleep(); // Wait until it's time for another iteration

   } while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok());

   /* for the final destination, adjust the orientation to match the goal pose */
   /* ------------------------------------------------------------------------ */

   if (!waypoints)
   {

     do
     {

         /* if the robot has reached the destination             */
         /* adjust the orientation to match the goal orientation */

         /* get the current pose */

         ros::spinOnce(); // Let ROS take over to handle the callback

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

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

         if (debug)
         {
      printf("Orienting\n");
      // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
      // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
      printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
      printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
         }

         is_moving = true;
         navigation_pelvis_publisher.publish(pelvis_msg);
         pub.publish(msg); // Publish the message
         // navigation_pelvis_publisher.publish(trajectory_msg);

         rate.sleep(); // Wait until it's time for another iteration

     } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

      current_linear_velocity = 0;
      msg.linear.x = current_linear_velocity;

      angular_velocity = 0;
      msg.angular.z = angular_velocity;

      navigation_pelvis_publisher.publish(pelvis_msg);
      pub.publish(msg); // Publish the message
      is_moving = false;
      // navigation_pelvis_publisher.publish(trajectory_msg);

      rate.sleep(); // Wait until it's time for another iteration

   }
 }


void goToPoseMIMO(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
 {

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;

   static double current_linear_velocity = 0; // make this static so that it's valid on the next call

   int number_of_ramp_up_steps = 20;

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

   while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok())
   {

     /* get the current pose */

     ros::spinOnce(); // Let ROS take over to handle the callback

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
     /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
     /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     /* set linear and angular velocities, taking care not to use values that exceed maximum values */
     /* or use values that are less than minimum values needed to produce a response in the robot   */

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     /* don't slow down if driving to a waypoint */

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

     /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

     if (current_linear_velocity == 0)
     {

         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
      msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
      msg.angular.z = 0;

      if (debug)
      {
              printf("Ramping up velocity\n");
              // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
              // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
              printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
              printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;

     if (debug)
     {
         printf("Going\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }

     pub.publish(msg); // Publish the message

     rate.sleep(); // Wait until it's time for another iteration

   }

   /* for the final destination, adjust the orientation to match the goal pose */
   /* ------------------------------------------------------------------------ */

   if (!waypoints)
   {
    angle_error = goal_theta - current_theta;

      while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok())
     {

         /* if the robot has reached the destination             */
         /* adjust the orientation to match the goal orientation */

         /* get the current pose */

         ros::spinOnce(); // Let ROS take over to handle the callback

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

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

         if (debug)
         {
      printf("Orienting\n");
      // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
      // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
      printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
      printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
         }

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration

     }

      current_linear_velocity = 0;
      msg.linear.x = current_linear_velocity;

      angular_velocity = 0;
      msg.angular.z = angular_velocity;

      pub.publish(msg); // Publish the message

      // rate.sleep(); // Wait until it's time for another iteration

   }
 }



void goToPoseMIMOFunctional(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
 {

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;

   static double current_linear_velocity = 0; // make this static so that it's valid on the next call

   int number_of_ramp_up_steps = 20;

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   do
   {

     /* get the current pose */

     ros::spinOnce(); // Let ROS take over to handle the callback

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
     /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
     /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     /* set linear and angular velocities, taking care not to use values that exceed maximum values */
     /* or use values that are less than minimum values needed to produce a response in the robot   */

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     /* don't slow down if driving to a waypoint */

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

     /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

     if (current_linear_velocity == 0)
     {

         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
      msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
      msg.angular.z = 0;

      if (debug)
      {
              printf("Ramping up velocity\n");
              // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
              // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
              printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
              printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;

     if (debug)
     {
         printf("Going\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }

     pub.publish(msg); // Publish the message

     rate.sleep(); // Wait until it's time for another iteration

   } while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok());

   /* for the final destination, adjust the orientation to match the goal pose */
   /* ------------------------------------------------------------------------ */

   if (!waypoints)
   {

     do
     {

         /* if the robot has reached the destination             */
         /* adjust the orientation to match the goal orientation */

         /* get the current pose */

         ros::spinOnce(); // Let ROS take over to handle the callback

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

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

         if (debug)
         {
      printf("Orienting\n");
      // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
      // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
      printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
      printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
         }

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration

     } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

      current_linear_velocity = 0;
      msg.linear.x = current_linear_velocity;

      angular_velocity = 0;
      msg.angular.z = angular_velocity;

      pub.publish(msg); // Publish the message

      // rate.sleep(); // Wait until it's time for another iteration

   }
 }


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
double roundFloatingPoint(double value, int decimal_places){
   double rounded_value = 0.0;

   // rounded_value = std::round(value * std::pow(10, decimal_places)) / std::pow(10, decimal_places);
   double multiplier = std::pow(10.0, decimal_places);
   rounded_value = std::roundf(value * multiplier) / multiplier;

   // rounded_value = static_cast<double>(rounded_value);

   return rounded_value;

}
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
double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to double
    return degrees;
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
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to double
    return radians;
}


/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void promptAndExit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void promptAndContinue(){
    printf("Press X to quit or Press any key to continue...\n");
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}

void moveRobotActuatorsToDefault(){
    double gesture_duration = 1.0; // Set the duration of the gesture to 2 seconds
    // Create a control client for the head
    std::string head_topic;
    head_topic = "/pepper_dcm/Head_controller/follow_joint_trajectory";
    ControlClientPtr head_client = createClient(head_topic);

    if(head_client == nullptr){
        ROS_ERROR("Error creating action client for head controller");
        return;
    }
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};// Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
   //  std::vector<double> head_position = {0.4325826168060303, -0.013805866241455078};
   std::vector<double> head_position = {0.0, 0.0};

    // Create a control client for the left arm
      std::string left_arm_topic;
      left_arm_topic = "/pepper_dcm/LeftArm_controller/follow_joint_trajectory";
      ControlClientPtr left_arm_client = createClient(left_arm_topic);

      if(left_arm_client == nullptr){
          ROS_ERROR("Error creating action client for left arm controller");
          return;
      }

      std::vector<std::string> left_arm_joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};// Set the joint names for the left arm to the specified joint names

      // positions for each joint
      std::vector<double> left_arm_position = {1.5523885488510132, 0.13959217071533203, -1.2087767124176025, -0.4540584087371826, -0.15497589111328125};
      
      // Create a control client for the right arm
      std::string right_arm_topic;
      right_arm_topic = "/pepper_dcm/RightArm_controller/follow_joint_trajectory";
      ControlClientPtr right_arm_client = createClient(right_arm_topic);

      if(right_arm_client == nullptr){
          ROS_ERROR("Error creating action client for right arm controller");
          return;
      }

      std::vector<std::string> right_arm_joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};// Set the joint names for the right arm to the specified joint names

      // positions for each joint
      std::vector<double> right_arm_position = {1.5447185039520264, -0.1441941261291504, 1.2133787870407104,  0.44945645332336426, 0.1487560272216797};
      
      // Create a control client for the leg
      std::string leg_topic;
      leg_topic = "//pepper_dcm/Pelvis_controller/follow_joint_trajectory";
      ControlClientPtr leg_client = createClient(leg_topic);

      if(leg_client == nullptr){
          ROS_ERROR("Error creating action client for leg controller");
          return;
      }

      std::vector<std::string> leg_joint_names = {"HipPitch", "HipRoll", "KneePitch"};// Set the joint names for the leg to the specified joint names

      // positions for each joint
      // std::vector<double> leg_position = {0.007669925689697266, -0.018407821655273438, 0.0490872859954834};
      std::vector<double> leg_position = {0.0, 0.0, 0.0};
    
    // Move the head to the specified position
    moveToPosition(head_client, head_joint_names, head_position, 
                     left_arm_client, left_arm_joint_names, left_arm_position, 
                     right_arm_client, right_arm_joint_names, right_arm_position, 
                     leg_client, leg_joint_names, leg_position, 
                     gesture_duration);

    // Move the head to the specified position
      // moveOneActuatorToPosition(head_client, head_joint_names, gesture_duration, head_position);

}

void moveToPosition(ControlClientPtr& head_client, const std::vector<std::string>& head_joint_names, std::vector<double> head_positions, 
                        ControlClientPtr& left_arm_client, const std::vector<std::string>& left_arm_joint_names, std::vector<double> left_arm_positions, 
                        ControlClientPtr& right_arm_client, const std::vector<std::string>& right_arm_joint_names, std::vector<double> right_arm_positions, 
                        ControlClientPtr& leg_client, const std::vector<std::string>& leg_joint_names, std::vector<double> leg_positions, 
                        double duration){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal head_goal;
    trajectory_msgs::JointTrajectory& head_trajectory = head_goal.trajectory;
    head_trajectory.joint_names = head_joint_names;                               // Set the joint names for the actuator to the specified joint names
    head_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    head_trajectory.points[0].positions = head_positions;                         // Set the positions in the trajectory to the specified positions
    head_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal left_arm_goal;
    trajectory_msgs::JointTrajectory& left_arm_trajectory = left_arm_goal.trajectory;
    left_arm_trajectory.joint_names = left_arm_joint_names;                               // Set the joint names for the actuator to the specified joint names
    left_arm_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    left_arm_trajectory.points[0].positions = left_arm_positions;                         // Set the positions in the trajectory to the specified positions
    left_arm_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal right_arm_goal;
    trajectory_msgs::JointTrajectory& right_arm_trajectory = right_arm_goal.trajectory;
    right_arm_trajectory.joint_names = right_arm_joint_names;                               // Set the joint names for the actuator to the specified joint names
    right_arm_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    right_arm_trajectory.points[0].positions = right_arm_positions;                         // Set the positions in the trajectory to the specified positions
    right_arm_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal leg_goal;
    trajectory_msgs::JointTrajectory& leg_trajectory = leg_goal.trajectory;
    leg_trajectory.joint_names = leg_joint_names;                               // Set the joint names for the actuator to the specified joint names
    leg_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    leg_trajectory.points[0].positions = leg_positions;                         // Set the positions in the trajectory to the specified positions
    leg_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Send the goal to move the actuator to the specified position
    // head_client->sendGoal(head_goal);
   left_arm_client->sendGoal(left_arm_goal);
   right_arm_client->sendGoal(right_arm_goal);
      leg_client->sendGoal(leg_goal);

    // head_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   left_arm_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   right_arm_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
      leg_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   // ros::Duration(duration).sleep();
}

void moveOneActuatorToPosition(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
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
