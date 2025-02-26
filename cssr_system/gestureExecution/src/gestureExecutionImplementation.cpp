/* gestureExecutionImplementation.cpp
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

#include "gestureExecution/gestureExecutionInterface.h"

/* --------------------------------------------------
            GLOBAL VARIABLES 
    -------------------------------------------------- 
*/

// Home positions for the robot
std::vector<double> right_arm_home_position = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};            // Shoulder pitch, shoulder roll, elbow roll, elbow yaw, wrist yaw, 
std::vector<double> left_arm_home_position = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};             // Shoulder pitch, shoulder roll, elbow roll, elbow yaw, wrist yaw
std::vector<double> leg_home_position = {0.0, 0.0, 0.0};                                                // Hip pitch, hip roll, knee pitch
std::vector<double> head_home_position = {-0.2, 0.0};                                                   // Head pitch and yaw

// Joint states of the robot - updated by subscribing to the /sensor_msgs/joint_states topic
std::vector<double> leg_joint_states = {0.0, 0.0, 0.0};
std::vector<double> head_joint_states = {0.0, 0.0};
std::vector<double> right_arm_joint_states = {0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> left_arm_joint_states = {0.0, 0.0, 0.0, 0.0, 0.0};

// Coordinates of the robot in the world (x, y, z, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};

ros::ServiceClient overt_attention_client;                                                              // Client for the overtAttention/set_mode service
cssr_system::setMode overt_attention_srv;                                                              // Service object for the overtAttention/set_mode service

std::string node_name;                                                                                  // Stores the name of the node

bool verbose_mode = false;                                                                              // Flag to indicate if verbose mode is enabled
string implementation_platform;                                                                         // Stores the implementation platform
string interpolation_type;                                                                              // Stores the interpolation type
string gesture_descriptors_config_filename;                                                             // Stores the gesture descriptors configuration filename
string simulator_topics_filename;                                                                       // Stores the simulator topics filename
string robot_topics_filename;                                                                           // Stores the robot topics filename
string verbose_mode_input;                                                                              // Stores the verbose mode input
string topics_file_name = "";                                                                           //Stores the name of the topics file to be used
int interpolation_mode = LINEAR_INTERPOLATION;                                                          // Stores the interpolation mode to be used

// Publisher for the velocity commands
ros::Publisher gesture_velocity_publisher;

bool first_service_call = true;                                                                        // Tracks the first call to the service request

bool shutdown_requested = false;                                                                       // Flag to indicate if a shutdown has been requested

bool node_initialized = false;                                                                         // Flag to indicate if the node has been initialized
// Iconic gestures descriptors table. Each row contains the gesture_id for both arms. Empty values means its only one arm gesture
std::vector<std::vector<string>> gesture_descriptors_table = {{"01", "02"},
                                                            {"02", "01"},
                                                            {"03", ""},
                                                            {"04", "05"},
                                                            {"05", "04"}};


/*  --------------------------------------------------
            CALLBACK FUNCTIONS 
    -------------------------------------------------- 
*/

/*
 *   Callback function for the joint states message received from the /sensor_msgs/joint_states topic
 *   The function receives the joint states message and stores the joint states of the robot
 */
void joint_states_message_received(const sensor_msgs::JointState& msg) {
    // Obtain an iterator for each joints
    auto head_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadPitch");
    auto head_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "HeadYaw");
    auto hip_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "HipPitch");
    auto hip_roll_iterator = std::find(msg.name.begin(), msg.name.end(), "HipRoll");
    auto knee_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "KneePitch");
    auto left_shoulder_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "LShoulderPitch");
    auto left_shoulder_roll_iterator = std::find(msg.name.begin(), msg.name.end(), "LShoulderRoll");
    auto left_elbow_roll_iterator = std::find(msg.name.begin(), msg.name.end(), "LElbowRoll");
    auto left_elbow_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "LElbowYaw");
    auto left_wrist_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "LWristYaw");
    auto right_shoulder_pitch_iterator = std::find(msg.name.begin(), msg.name.end(), "RShoulderPitch");
    auto right_shoulder_roll_iterator = std::find(msg.name.begin(), msg.name.end(), "RShoulderRoll");
    auto right_elbow_roll_iterator = std::find(msg.name.begin(), msg.name.end(), "RElbowRoll");
    auto right_elbow_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "RElbowYaw");
    auto right_wrist_yaw_iterator = std::find(msg.name.begin(), msg.name.end(), "RWristYaw");

    // Get the index of each joint in the joint states message
    int head_pitch_index = std::distance(msg.name.begin(), head_pitch_iterator);
    int head_yaw_index = std::distance(msg.name.begin(), head_yaw_iterator);
    int hip_pitch_index = std::distance(msg.name.begin(), hip_pitch_iterator);
    int hip_roll_index = std::distance(msg.name.begin(), hip_roll_iterator);
    int knee_pitch_index = std::distance(msg.name.begin(), knee_pitch_iterator);
    int left_shoulder_pitch_index = std::distance(msg.name.begin(), left_shoulder_pitch_iterator);
    int left_shoulder_roll_index = std::distance(msg.name.begin(), left_shoulder_roll_iterator);
    int left_elbow_roll_index = std::distance(msg.name.begin(), left_elbow_roll_iterator);
    int left_elbow_yaw_index = std::distance(msg.name.begin(), left_elbow_yaw_iterator);
    int left_wrist_yaw_index = std::distance(msg.name.begin(), left_wrist_yaw_iterator);
    int right_shoulder_pitch_index = std::distance(msg.name.begin(), right_shoulder_pitch_iterator);
    int right_shoulder_roll_index = std::distance(msg.name.begin(), right_shoulder_roll_iterator);
    int right_elbow_roll_index = std::distance(msg.name.begin(), right_elbow_roll_iterator);
    int right_elbow_yaw_index = std::distance(msg.name.begin(), right_elbow_yaw_iterator);
    int right_wrist_yaw_index = std::distance(msg.name.begin(), right_wrist_yaw_iterator);

    // Store the joint states in the respective arrays
    head_joint_states[0] = msg.position[head_pitch_index];
    head_joint_states[1] = msg.position[head_yaw_index];

    leg_joint_states[0] = msg.position[hip_pitch_index];
    leg_joint_states[1] = msg.position[hip_roll_index];
    leg_joint_states[2] = msg.position[knee_pitch_index];

    left_arm_joint_states[0] = msg.position[left_shoulder_pitch_index];
    left_arm_joint_states[1] = msg.position[left_shoulder_roll_index];
    left_arm_joint_states[2] = msg.position[left_elbow_roll_index];
    left_arm_joint_states[3] = msg.position[left_elbow_yaw_index];
    left_arm_joint_states[4] = msg.position[left_wrist_yaw_index];

    right_arm_joint_states[0] = msg.position[right_shoulder_pitch_index];
    right_arm_joint_states[1] = msg.position[right_shoulder_roll_index];
    right_arm_joint_states[2] = msg.position[right_elbow_roll_index];
    right_arm_joint_states[3] = msg.position[right_elbow_yaw_index];
    right_arm_joint_states[4] = msg.position[right_wrist_yaw_index];
}

/*
 *   Callback function for the robot pose message received from the /robotLocalization/pose topic
 *   The function receives the robot pose message and stores the pose of the robot
 */
void robot_pose_message_received(const geometry_msgs::Pose2D& msg) {
    // Store the robot pose in the robot_pose array
    robot_pose[0] = msg.x;
    robot_pose[1] = msg.y;
    robot_pose[2] = msg.theta;
}

/* 
 *   Callback function for the /gestureExecution/perform_gesture service
 *   The function receives a request to execute a gesture on the robot
 *   and executes the gesture based on the request parameters.
 */
bool execute_gesture(cssr_system::performGesture::Request  &service_request, cssr_system::performGesture::Response &service_response){
    int execution_status = 0;                                                       //Stores the status of the gesture execution
    bool debug = true;                                                              //Debug mode flag

    service_response.gesture_success = execution_status;                            // Set the default response to the service

    if(first_service_call){                                                        // Check if this is the first call to the service
        // Initialize the node: Read configuration file and set the implementation platform, interpolation type, and topics file name
        int node_initialization_status = 0;
        node_initialization_status = initialize_node(&implementation_platform, &interpolation_type, &interpolation_mode, &gesture_descriptors_config_filename, &simulator_topics_filename, &robot_topics_filename, &topics_file_name, &verbose_mode_input, &verbose_mode);
        if(node_initialization_status != 0){
            ROS_ERROR("%s: initialization failed due to incorrect configuration. Terminating...", node_name.c_str());
            return 0;
        }
        first_service_call = false;
    }

    // Print the request parameters if verbose mode is enabled
    if (verbose_mode){
        ROS_INFO("%s: request to /gestureExecution/perform_gesture service: \
                \n\t\t\t\t\t\t\tgesture_type\t\t: %s, \n\t\t\t\t\t\t\tgesture_id\t\t: %d, \n\t\t\t\t\t\t\tgesture_duration\t: %ld ms, \
                \n\t\t\t\t\t\t\tbow_nod_angle\t\t: %d degrees, \
                \n\t\t\t\t\t\t\tlocation_x\t\t: %.2f, \n\t\t\t\t\t\t\tlocation_y\t\t: %.2f, \n\t\t\t\t\t\t\tlocation_z\t\t: %.2f.",\
                node_name.c_str(), service_request.gesture_type.c_str(), service_request.gesture_id, service_request.gesture_duration, \
                service_request.bow_nod_angle, service_request.location_x, service_request.location_y, service_request.location_z);
    }

    // Extract the request parameters
    string gesture_type = service_request.gesture_type;
    uint8_t gesture_id = service_request.gesture_id;
    int gesture_duration = service_request.gesture_duration;
    int bow_nod_angle = service_request.bow_nod_angle;
    float point_location_x = service_request.location_x;
    float point_location_y = service_request.location_y;
    float point_location_z = service_request.location_z;

    // Ensure the request gesture duration is not less than 1000 ms or greater than 10000 ms -- Values defined in gestureExecutionINterface.h
    if (gesture_duration < MIN_GESTURE_DURATION){
        ROS_WARN("%s: gesture duration cannot be less than %d ms. Requested gesture duration: %d ms", node_name.c_str(), MIN_GESTURE_DURATION, gesture_duration);
        gesture_duration = MIN_GESTURE_DURATION;
    }
    else if (gesture_duration > MAX_GESTURE_DURATION){
        ROS_WARN("%s: gesture duration cannot be greater than %d ms. Requested gesture duration: %d ms", node_name.c_str(), MAX_GESTURE_DURATION, gesture_duration);
        gesture_duration = MAX_GESTURE_DURATION;
    }

    /* -----Main gesture execution logic------ */
 
    /* Deictic Gestures Execution */
    if((gesture_type == DEICTIC_GESTURES) || (gesture_type == DIECTIC_GESTURES)){
        // Execute the deictic gesture
        execution_status = deictic_gesture(point_location_x, point_location_y, point_location_z, gesture_duration, topics_file_name, interpolation_mode, gesture_velocity_publisher, verbose_mode);
    }

    /* Iconic Gestures Execution */
    else if(gesture_type == ICONIC_GESTURES){
        string iconic_gesture_type;                                                 //Stores the iconic gesture type
        string iconic_gesture_id;                                                   //Stores the iconic gesture ID
        string gesture_arm = "RArm";                                                //Stores the gesture arm
        int number_of_waypoints;                                                    //Stores the number of waypoints in the gesture
        string joint_angles;                                                        //Stores the joint angles of the gesture

        // Vectors to store the waypoints of the gesture
        std::vector<std::vector<double>> gesture_waypoints;
        std::vector<std::vector<double>> gesture_waypoints_arm_1;
        std::vector<std::vector<double>> gesture_waypoints_arm_2;

        // Read gesture descriptors configuration file
        std::vector<std::vector<string>> gesture_descriptors_config;
        int gesture_descriptors_config_read_status = 0;
        gesture_descriptors_config_read_status = read_gesture_descriptors_config(gesture_descriptors_config_filename, gesture_descriptors_config);
        if(gesture_descriptors_config_read_status != 0){
            ROS_ERROR("%s: error reading gesture descriptors configuration file.", node_name.c_str());
            return true;
        }

        // Analyse the requested ID to determine if it is one or two-armed gesture from the descriptor table
        bool ID_found = false;                                                      //Stores the status of the ID found
        string second_arm_ID = "";                                                  //Stores the ID of the second arm
        for (int i = 0; i < gesture_descriptors_table.size(); i++){                 //Iterate through the gesture descriptors table
            if (gesture_id == std::stoi(gesture_descriptors_table[i][0])){          //Check if the gesture ID is in the table
                second_arm_ID = gesture_descriptors_table[i][1];                    //Get the second arm ID
                ID_found = true;                                                    //Set the ID found flag
                break;                                                              //Exit the loop
            }
        }

        if (!ID_found){                                                             //If the ID is not found in the table
            ROS_WARN("%s: gesture ID not found in the gesture descriptors table.", node_name.c_str());
            ROS_ERROR("%s: executing service request failed.", node_name.c_str());
            service_response.gesture_success = 0;                                  // Set the response to the service
            return true;
        }

        string gesture_arm_1 = "";                                                  //Stores the gesture arm 1
        string gesture_arm_2 = "";                                                  //Stores the gesture arm 2
        string gesture_descriptors_filename = "";                                   //Stores the gesture descriptors filename
        int number_of_waypoints_arm_1 = 0;                                          //Stores the number of waypoints for arm 1
        int number_of_waypoints_arm_2 = 0;                                          //Stores the number of waypoints for arm 2                                       
        string joint_angles_arm_1 = "";                                             //Stores the joint angles for arm 1
        string joint_angles_arm_2 = "";                                             //Stores the joint angles for arm 2

        int angles_extracted_arm_1 = 1;                                             //Stores the status of the joint angles extraction for arm 1
        int angles_extracted_arm_2 = 1;                                             //Stores the status of the joint angles extraction for arm 2

        bool open_right_hand = true;                                                //Stores the status of the right hand opening
        bool open_left_hand = false;                                                //Stores the status of the left hand opening

        // Extract the gesture descriptors from the gesture ID
        extract_info_from_ID(gesture_id, gesture_descriptors_config, &gesture_descriptors_filename, &gesture_arm_1);

        // Read the gesture descriptors file. If an error occurs, return true
        if(read_gesture_descriptors(gesture_descriptors_filename, &iconic_gesture_type, &iconic_gesture_id, &number_of_waypoints_arm_1, &joint_angles_arm_1)){
            ROS_ERROR("%s: error reading gesture descriptors file.", node_name.c_str());
            return true;
        }

        // Extract joint angles from the gesture descriptors for arm 1
        angles_extracted_arm_1 = extract_joint_angles(gesture_descriptors_filename, joint_angles_arm_1, number_of_waypoints_arm_1, gesture_waypoints_arm_1, verbose_mode);

        // If the gesture is a two-armed gesture, extract the joint angles for the second arm
        if (second_arm_ID != ""){
            extract_info_from_ID(std::stoi(second_arm_ID), gesture_descriptors_config, &gesture_descriptors_filename, &gesture_arm_2);
            if(read_gesture_descriptors(gesture_descriptors_filename, &iconic_gesture_type, &iconic_gesture_id, &number_of_waypoints_arm_2, &joint_angles_arm_2)){
                ROS_ERROR("%s: error reading gesture descriptors file.", node_name.c_str());
                return true;
            }
            // Extract joint angles from the gesture descriptors
            angles_extracted_arm_2 = extract_joint_angles(gesture_descriptors_filename, joint_angles_arm_2, number_of_waypoints_arm_2, gesture_waypoints_arm_2, verbose_mode);
        
        }

        // Execute the iconic gesture
        if(angles_extracted_arm_1 && angles_extracted_arm_2){
            execution_status = iconic_gestures(gesture_arm_1, gesture_waypoints_arm_1, gesture_arm_2, gesture_waypoints_arm_2, open_right_hand, open_left_hand, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
        }
    }

    /* Symbolic Gestures Execution */
    else if((gesture_type == SYMBOLIC_GESTURES)){
        ROS_WARN("%s: symbolic gesture execution not implemented yet.", node_name.c_str());
        service_response.gesture_success = 0;                                      // Set the response to the service
        return true;
    }
    
    /* Bowing Gesture Execution */
    else if(gesture_type == BOWING_GESTURE){
        execution_status = bowing_gesture(bow_nod_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
    }

    /* Nodding Gesture Execution */
    else if(gesture_type == NODDING_GESTURE){
        execution_status = nodding_gesture(bow_nod_angle, gesture_duration, topics_file_name, interpolation_mode, verbose_mode);
    }

    /* Gesture type not supported */
    else{
        ROS_WARN("%s: gesture type not supported. Supported gesture types are: deictic, iconic, symbolic, bow, and nod.", node_name.c_str());
    }

    service_response.gesture_success = execution_status;                            // Set the response to the service

    if (service_response.gesture_success == 1){                                     // Print the response to the service
        if(verbose_mode){ 
            ROS_INFO("%s: executing service request successful.", node_name.c_str());
        }
    }
    else{
        ROS_ERROR("%s: executing service request failed.", node_name.c_str());
    }
    return true;
}




/*  --------------------------------------------------
            CONFIGURATION CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to read the the robot pose from an input file -- A driver just in case robotLocalization is not available
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;                                                        // used to turn debug message on

    std::string data_file = "robotPose.dat";                                        // data filename
    std::string data_path;                                                          // data path
    std::string data_path_and_file;                                                 // data path and filename

    std::string x_key = "x";                                                        // x key
    std::string y_key = "y";                                                        // y key
    std::string theta_key = "theta";                                                // theta key

    std::string x_value;                                                            // x value
    std::string y_value;                                                            // y value
    std::string z_value;                                                            // z value
    std::string theta_value;                                                        // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += DATA_FILE_PATH;
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) ROS_INFO("%s: robot pose input data file is %s", node_name.c_str(), data_path_and_file.c_str());

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("%s: unable to open the robot pose input data file %s", node_name.c_str(), data_path_and_file.c_str());
    }

    std::string data_line_read;                                                     // variable to read the line in the file
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
    bool topic_available = false;                                                   // boolean to store if the topic is available
    ros::master::V_TopicInfo master_topics;                                         // vector to store the topics
    ros::master::getTopics(master_topics);                                          // get the topics

    // Iterate through the topics to check if the topic is available
    for (const auto& topic : master_topics){
        if (topic.name == topic_name){                                              // if the topic is found
            topic_available = true;                                                 // set the topic as available
            break;
        }
    }

    return topic_available;                                                         // return the topic availability
}

/* 
 *   Function to verify if a service is available
 * @param:
 *   service_name: string to store the service name
 * 
 * @return:
 *  boolean indicating if the service is available
 */
bool is_service_available(const std::string& service_name){
    if(ros::service::exists(service_name, false)){                                  // check if the service exists
        return true;                                                                // return true if the service exists
    }
    return false;                                                                   // return false if the service does not exist
}

/* 
 *   Function to initialize the /gestureExecution node.
 *   The service is used to execute gestures on the robot. This initialization reads the gesture execution configuration file
 *   and sets the parameters for the gesture execution service.
 *
 * @param:
 *   implementation_platform: string to store the implementation platform
 *   interpolation_type: string to store the interpolation type
 *   interpolation_mode: integer to store the interpolation mode
 *   gesture_descriptors_config_filename: string to store the gesture descriptors configuration filename
 *   simulator_topics_filename: string to store the simulator topics filename
 *   robot_topics_filename: string to store the robot topics filename
 *   topics_filename: string to store the topics filename
 *   verbose_mode_input: string to store the verbose mode input
 *   verbose_mode: boolean to store the verbose mode
 *
 * @return:
 *   0 if successful, 1 otherwise
 */
int initialize_node(string *implementation_platform, string *interpolation_type, int* interpolation_mode, string *gesture_descriptors_config_filename, string *simulator_topics_filename, string *robot_topics_filename, string *topics_filename, string *verbose_mode_input, bool* verbose_mode){
    /* Read gesture execution configuration */
    int configuration_read;
    if(read_gesture_execution_configuration(implementation_platform, interpolation_type, gesture_descriptors_config_filename, simulator_topics_filename, robot_topics_filename, verbose_mode_input)){
        return 1;                                                                   // return 1 if the gesture execution configuration is not successful
    }

    // Set the verbose mode
    if (*verbose_mode_input == "true"){
        *verbose_mode = true;
    }
    else if (*verbose_mode_input == "false"){
        *verbose_mode = false;
    }
    else{
        ROS_ERROR("%s: verboseMode value in gestureExecutionConfiguration.ini file not supported. Supported verboseMode values are: true and false.", node_name.c_str());
        return 1;                                                                   // return 1 if the verbose mode is not supported
    }

    // Print the gesture execution configuration
    ROS_INFO("%s: configuration parameters: \
                \n\t\t\t\t\t\t\tImplementation platform\t\t: %s, \n\t\t\t\t\t\t\tInterpolation type\t\t: %s, \
                \n\t\t\t\t\t\t\tGestures Descriptors file\t: %s, \n\t\t\t\t\t\t\tSimulator Topics file\t\t: %s, \
                \n\t\t\t\t\t\t\tRobot Topics file\t\t: %s, \n\t\t\t\t\t\t\tVerbose Mode\t\t\t: %s.",\
                node_name.c_str(), implementation_platform->c_str(), interpolation_type->c_str(), gesture_descriptors_config_filename->c_str(), \
                simulator_topics_filename->c_str(), robot_topics_filename->c_str(), verbose_mode_input->c_str());

    /* Check if the implementation platform is supported and set the topics file name */
    {
        if (*implementation_platform == "simulator"){
            *topics_filename = *simulator_topics_filename;
        }
        else if (*implementation_platform == "robot"){
            *topics_filename = *robot_topics_filename;
        }
        else{
            ROS_ERROR("%s: implementation platform in gestureExecutionConfiguration.ini not supported. Supported implementation platforms are: simulator and robot.", node_name.c_str());
            return 1;                                                               // return 1 if the implementation platform is not supported
        }
    }

    /* Check if the interpolation type is supported and set the interpolation mode */
    {
        if (*interpolation_type == "linear"){
            *interpolation_mode = LINEAR_INTERPOLATION;
        }
        else if (*interpolation_type == "biological"){
            *interpolation_mode = BIOLOGICAL_MOTION;
        }
        else{
            ROS_ERROR("%s: interpolation type in gestureExecutionConfiguration.ini not supported. Supported interpolation types are: linear and biological_motion.", node_name.c_str());
            return 1;                                                               // return 1 if the interpolation type is not supported
        }
    }

    return 0;                                                                      // return 0 if successful
}

/* 
 *   Function to read the gesture execution configuration.
 *   The configuration file contains the platform, interpolation type, gesture descriptors, simulator topics, robot topics and verbose mode.
 *
 * @param:
 *   platform: string to store the platform
 *   interpolation: string to store the interpolation type
 *   gesture_descriptors: string to store the gesture descriptors filename
 *   simulator_topics: string to store the simulator topics filename
 *   robot_topics: string to store the robot topics filename
 *   verbose_mode: string to store the verbose mode
 *
 * @return:
 *   0 if successful, 1 otherwise
 */
int read_gesture_execution_configuration(string* platform, string* interpolation, string* gesture_descriptors, string* simulator_topics, string* robot_topics, string* verbose_mode){
    bool debug_mode = false;                                                        // used to turn debug message on  

    std::string config_file = "gestureExecutionConfiguration.ini";                  // data filename
    std::string config_path;                                                        // data path
    std::string config_path_and_file;                                               // data path and filename
     
    std::string platform_key = "platform";                                          // platform key 
    std::string interpolation_key = "interpolation";                                // interpolation key key
    std::string gesture_descriptors_key = "gestureDescriptors";                     // gesture descriptors key
    std::string simulator_topics_key = "simulatorTopics";                           // simulator topics key
    std::string robot_topics_key = "robotTopics";                                   // robot topics key
    std::string verbose_mode_key = "verboseMode";                                   // verbose mode key

    std::string platform_value;                                                     // platform key 
    std::string interpolation_value;                                                // interpolation key key
    std::string gesture_descriptors_value;                                          // gesture descriptors key
    std::string simulator_topics_value;                                             // simulator topics key
    std::string robot_topics_value;                                                 // robot topics key
    std::string verbose_mode_value;                                                 // verbose mode key

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += CONFIGURATION_FILE_PATH;
    config_path_and_file = config_path;
    config_path_and_file += config_file;

    if (debug_mode) ROS_INFO("%s: configuration file is %s", node_name.c_str(), config_path_and_file.c_str());

    // Open configuration file
    std::ifstream config_if(config_path_and_file.c_str());
    if (!config_if.is_open()){
        ROS_ERROR("%s: unable to open the configuration file %s", node_name.c_str(), config_path_and_file.c_str());
        return 1;                                                                   // return 1 if the configuration file is not opened
    }

    // Set platform to the default value of robot
    *platform = "robot";

    std::string data_line_read;                                                     // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(config_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;                                                           // read the key
        trim(param_key);                                                            // remove leading and trailing whitespaces
        std::getline(iss, param_value);                                             // read the value
        iss >> param_value;                                                         // read the value
        trim(param_value);                                                          // remove leading and trailing whitespaces
        
        // Extract the platform -- Removed the platform key from the configuration file but will be useful if you need it
        if (param_key == platform_key){ 
            boost::algorithm::to_lower(param_value);                                // modifies string to lower case
            platform_value = param_value;
            *platform = param_value;
        }
        // Extract the interpolation type
        else if (param_key == interpolation_key){ 
            interpolation_value = param_value;
            boost::algorithm::to_lower(param_value);                                // modifies string to lower case
            *interpolation = param_value;
        }
        // Extract the gesture descriptors filename
        else if (param_key == gesture_descriptors_key){ 
            gesture_descriptors_value = param_value;
            *gesture_descriptors = param_value;
        }
        // Extract the simulator topics filename
        else if (param_key == simulator_topics_key){ 
            simulator_topics_value = param_value;
            *simulator_topics = param_value;
        }
        // Extract the robot topics filename
        else if (param_key == robot_topics_key){ 
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }
        // Extract the verbose mode
        else if (param_key == verbose_mode_key){ 
            boost::algorithm::to_lower(param_value);                                // modifies string to lower case
            verbose_mode_value = param_value;
            *verbose_mode = param_value;
        }
    }
    // Close the configuration file
    config_if.close();

    return 0;                                                                       // return 0 if successful
}

/*   
 *   Function to extract the gesture descriptors from the gesture descriptors file.
 *   The gesture descriptors file contains the gesture type, gesture ID, number of waypoints and joint angles.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors filename
 *       gesture_type: string to store the gesture type
 *       gesture_id: string to store the gesture ID
 *       number_of_waypoints: integer to store the number of waypoints
 *       joint_angles: string to store the joint angles
 *
 *   @return:
 *       0 if successful, 1 otherwise
 */
int read_gesture_descriptors(string gesture_descriptors_file, string* gesture_type, string* gesture_id, int* number_of_waypoints, string* joint_angles){
    bool debug_mode = false;   // used to turn debug message on  

    std::string data_file = gesture_descriptors_file;                               // data filename
    std::string data_path;                                                          // data path
    std::string data_path_and_file;                                                 // data path and filename
     
    std::string gesture_type_key = "type";                                          // gesture type key
    std::string gesture_ID_key = "ID";                                              // gesture ID key
    std::string waypoints_key = "wayPoints";                                        // way point key
    std::string joint_angles_key = "jointAngles";                                   // joint angles key

    std::string gesture_type_value;                                                 // gesture type value
    std::string gesture_ID_value;                                                   // gesture ID value
    std::string waypoints_value;                                                    // way point value
    std::string joint_angles_value;                                                 // joint angles value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += DATA_FILE_PATH;
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) ROS_INFO("%s: gesture descriptors file is %s", node_name.c_str(), data_path_and_file.c_str());

    // Open descriptors file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("%s: unable to open the gesture descriptors file %s", node_name.c_str(), data_path_and_file.c_str());
        return 1;                                                                   // return 1 if the data file is not opened
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        // Extract the gesture type        
        if (param_key == gesture_type_key){ 
            gesture_type_value = param_value;
            *gesture_type = param_value;
        }
        // Extract the gesture ID
        else if (param_key == gesture_ID_key){ 
            gesture_ID_value = param_value;
            *gesture_id = param_value;
        }
        // Extract the number of waypoints
        else if (param_key == waypoints_key){ 
            waypoints_value = param_value;
            *number_of_waypoints = std::stoi(param_value);
        }
        // Extract the joint angles
        else if (param_key == joint_angles_key){ 
            joint_angles_value = param_value;
            *joint_angles = param_value;
        }
    }
    // Close the descriptors file
    data_if.close();

    return 0;                                                                       // return 0 if successful
}

/*  
 *   Function to read the gesture descriptors configuration from the gesture descriptors configuration file.
 *   The gesture descriptors configuration file contains the gesture ID, gesture arm and gesture descriptors filename.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors configuration filename
 *       gesture_descriptors_config: vector to store the gesture descriptors configuration
 *
 *   @return:
 *       None
 */
int read_gesture_descriptors_config(string gesture_descriptors_file, std::vector<std::vector<string>>& gesture_descriptors_config){
    bool debug_mode = false;   // used to turn debug message on  

    std::string data_file = gesture_descriptors_file;                               // data filename
    std::string data_path;                                                          // data path
    std::string data_path_and_file;                                                 // data path and filename
     
    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += DATA_FILE_PATH;
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) ROS_INFO("%s: gesture descriptors configuration file is %s", node_name.c_str(), data_path_and_file.c_str());

    // Open descriptors file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("%s: unable to open the gesture descriptors configuration  file %s", node_name.c_str(), data_path_and_file.c_str());
        return 1;
    }

    // Clear the gesture descriptors configuration
    gesture_descriptors_config.clear();
    
    std::string data_line_read;                                                     // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string ID;
        std::string gesture_arm;
        std::string descriptors_filename;

        // Extract the gesture ID, gesture arm and gesture descriptors filename
        if(iss >> ID >> gesture_arm >> descriptors_filename){
            // Remove leading and trailing whitespaces
            trim(ID);                       
            trim(gesture_arm);
            trim(descriptors_filename);

            // Store the gesture descriptors configuration
            std::vector<string> gesture_descriptors;
            gesture_descriptors.push_back(ID);
            gesture_descriptors.push_back(gesture_arm);
            gesture_descriptors.push_back(descriptors_filename);

            // Add the gesture descriptors configuration to the vector
            gesture_descriptors_config.push_back(gesture_descriptors);
        }
        else{
            ROS_ERROR("%s: error reading gesture descriptors configuration file %s. Invalid format.", node_name.c_str(), data_path_and_file.c_str());
            return 1;    
        }
    }
    // Close the descriptors file
    data_if.close();
    return 0;
}

/*  
 *   Function to extract the gesture arm and gesture descriptors filename from the gesture ID.
 *   The gesture descriptors configuration file contains the gesture ID, gesture arm and gesture descriptors filename.
 *
 *   @param:
 *       gesture_id: the gesture ID
 *       gesture_descriptors_config: vector to store the gesture descriptors configuration
 *       gesture_descriptors_filename: string to store the gesture descriptors filename
 *       gesture_arm: string to store the gesture arm
 *
 *   @return:
 *       None
 */
void extract_info_from_ID(uint8_t gesture_id, std::vector<std::vector<string>> gesture_descriptors_config, string* gesture_descriptors_filename, string* gesture_arm){
    // Obtain correct gesture descriptor file. Iterate through the gesture descriptors configuration to get the gesture arm and gesture descriptors filename
    for (int i = 0; i < gesture_descriptors_config.size(); i++){
        if (gesture_id == std::stoi(gesture_descriptors_config[i][0])){
            *gesture_arm = gesture_descriptors_config[i][1];
            *gesture_descriptors_filename = gesture_descriptors_config[i][2];
            break;                                                                  // break the loop once the details for gesture ID is found
        }
    }
}


/*  
 *   Function to extract the joint angles from the vector of waypoints specidfied in the gesture descriptors file.
 *   The joint angles are extracted from the gesture descriptors file.
 *
 *   @param:
 *       gesture_descriptors_file: the gesture descriptors filename
 *       joint_angles: string to store the joint angles
 *       number_of_waypoints: integer to store the number of waypoints
 *       waypoints: vector to store the waypoints
 *       verbose_mode: boolean to store the verbose mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int extract_joint_angles(string gesture_descriptors_file, string joint_angles, int number_of_waypoints, std::vector<std::vector<double>>& waypoints, bool verbose_mode){

    // CLear the waypoints vector
    waypoints.clear();

    // Split the joint angles into individual waypoints
    std::vector<std::string> joint_angles_list;
    boost::split(joint_angles_list, joint_angles, boost::is_any_of(";"));

    if (number_of_waypoints != joint_angles_list.size()){
        if(verbose_mode){
            ROS_ERROR("%s: the number of waypoints specified '%d' in the gesture descriptor file '%s' does not match the number joint angles vectors '%lu'.", node_name.c_str(), number_of_waypoints, gesture_descriptors_file.c_str(), joint_angles_list.size());
        }
        return 0;
    }

    // Extract the joint angles for each waypoint
    for(int i = 0; i < number_of_waypoints; i++){
        std::vector<double> waypoint;
        std::vector<std::string> joint_angle;
        boost::split(joint_angle, joint_angles_list[i], boost::is_any_of(" "));
        for(int j = 0; j < joint_angle.size(); j++){
            waypoint.push_back(std::stod(joint_angle[j]));
        }
        waypoints.push_back(waypoint);
    }

    return 1;
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
int extract_topic(string key, string topic_file_name, string *topic_name){
    bool debug = false;                                                             // used to turn debug message on
    
    std::string topic_path;                                                         // topic filename path
    std::string topic_path_and_file;                                                // topic with path and file 

    std::string topic_value = "";                                                   // topic value

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += DATA_FILE_PATH;
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) ROS_INFO("%s: topic file is %s", node_name.c_str(), topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("%s: unable to open the topic file %s", node_name.c_str(), topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;                                                    // variable to read the line in the file
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
        if (param_key == key) {                                                     // if the key is found
            topic_value = param_value;                                              // set the topic value
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        ROS_ERROR("%s: unable to find a valid topic for '%s'. Please check the topics file %s.", node_name.c_str(), key.c_str(), topic_path_and_file.c_str());
        return 1;
    }

    *topic_name = topic_value;                                                      // set the topic name
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
    int max_iterations = 5;                                                        // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(1.0))) {
            return actionClient;                                                    // return the action client if the server is available
        }
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for the %s controller to come up", node_name.c_str(), topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    ROS_ERROR("%s: error creating action client for %s controller: Server not available", node_name.c_str(), topic_name.c_str());
    return NULL;                                                                    // return the null if the server is not available
}

/* 
 *   Function to return all joints of an actuator to the home position.
 *   This function is called on only one actuator at a time.
 *   It used the global variable set above for the home positions of the actuators.
 *
 * @param:
 *   actuator: string indicating the actuator to move to the home position
 *   topics_filename: string indicating the topics filename
 *   interpolation: integer indicating the interpolation type
 *   debug: boolean indicating the debug mode
 *
 * @return:
 *   None
 */
int go_to_home(std::string actuator, std::string topics_filename, int interpolation, bool debug){
    // ros::Duration(0.5).sleep(); // Wait for one second to ensure that the joint states are updated
    std::vector<double> actuator_state;                                             // stores the current state of the actuator joints
    std::vector<double> actuator_home_position;                                     // stores the home position of the actuator joints
    ControlClientPtr actuator_client;                                               // action client to control the actuator joints
    std::vector<std::string> actuator_joint_names;                                  // stores the joint names of the actuator joints
    std::string actuator_topic;                                                     // stores the topic of the actuator for control
    int number_of_joints;                                                           // stores the number of joints of the actuator
    double home_duration;                                                           // stores the duration to move to the home position

    // Set the open hand flag and the hand topic. ust for default, the hand is not to be open in home position
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Extract the actuator topic
    if(extract_topic(actuator, topics_filename, &actuator_topic)){
        return 0;
    }

    // Set the home duration
    home_duration = 1.0;

    // Set the actuator state, home position, joint names, client and number of joints based on the actuator
    if(actuator == "RArm"){                                                         // Right arm
        actuator_state = right_arm_joint_states;
        actuator_home_position = right_arm_home_position;
        actuator_joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
        hand = "RHand";
        if(extract_topic(hand, topics_filename, &hand_topic)){
            return 0;
        }
    }
    else if(actuator == "LArm"){                                                    // Left arm
        actuator_state = left_arm_joint_states;
        actuator_home_position = left_arm_home_position;
        actuator_joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();

        hand = "LHand";
        if(extract_topic(hand, topics_filename, &hand_topic)){
            return 0;
        }
    }
    else if(actuator == "Leg"){                                                     // Leg
        actuator_state = leg_joint_states;
        actuator_home_position = leg_home_position;
        actuator_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }
    else if(actuator == "Head"){                                                    // Head
        actuator_state = head_joint_states;
        actuator_home_position = head_home_position;
        actuator_joint_names = {"HeadPitch", "HeadYaw"};
        actuator_client = create_client(actuator_topic);
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

    // Compute the trajectory to get to the home position
    compute_trajectory(actuator_state, actuator_home_position, actuator_state.size(), home_duration, positions_t, velocities_t, accelerations_t, duration_t);

    // Move the joints of the actuator depending on the interpolation means selected
    if(interpolation == BIOLOGICAL_MOTION){                                         // Biological motion interpolation
        move_to_position_biological_motion(actuator_client, actuator_joint_names, duration_t, home_duration, open_hand, hand, hand_topic, "home", positions_t, velocities_t, accelerations_t);
    }
    else{                                                                           // Linear interpolation
        move_to_position(actuator_client, actuator_joint_names, home_duration, open_hand, hand, hand_topic, "home", actuator_home_position);
    }

    return 1;


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
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void head_pointing(std::string head_topic, double head_pitch, double head_yaw, double gesture_duration, int interpolation, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    if(head_client == NULL){
        return;
    }
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};           // Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size();                                 // Set the number of joints in the head to the size of the joint names
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    // Variables to control the hand, not necessary for the head, thus kept at false
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Move the head to the specified position
    if(interpolation == BIOLOGICAL_MOTION){                                         // If the interpolation type is biological motion
        // ros::Duration(5.0).sleep();
        // Compute the trajectory for the head
        compute_trajectory(head_home_position, head_position, number_of_joints, 1.0, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);
        // Move the head
        move_to_position_biological_motion(head_client, head_joint_names, duration_t_head, 0.25, open_hand, hand, hand_topic, "point", positions_t_head, velocities_t_head, accelerations_t_head);
    }
    else if(interpolation == LINEAR_INTERPOLATION){                                 // If the interpolation type is linear interpolation
        // Move the head to the specified position
        move_to_position(head_client, head_joint_names, 1.0, open_hand, hand, hand_topic, "point", head_position);
    }
}

/*  
 *   Function to move the right arm to point in a particular direction as specified by the joint angles
 *   The function moves the right arm to point in the specified direction using the control client
 *
 *   @param:
 *       right_arm_topic: the topic for the right arm
 *       shoulder_pitch: the pitch angle of the shoulder
 *       shoulder_roll: the roll angle of the shoulder
 *       elbow_yaw: the yaw angle of the elbow
 *       elbow_roll: the roll angle of the elbow
 *       wrist_yaw: the yaw angle of the wrist
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void right_arm_pointing(std::string right_arm_topic, double shoulder_pitch, double shoulder_roll, double elbow_yaw, double elbow_roll, double wrist_yaw, double gesture_duration, int interpolation, bool debug){
    // Create a control client for the right arm
    ControlClientPtr right_arm_client = create_client(right_arm_topic);
    if(right_arm_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    int number_of_joints = joint_names.size();                                      // Set the number of joints in the right arm to the size of the joint names
    
    // The point position for the right arm
    std::vector<double> point_position = {shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw};

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Variables to control the hand. Necessary for the right arm, thus set to true
    bool open_hand = true;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Move the right arm to point in the specified direction
    if(interpolation == BIOLOGICAL_MOTION){                                         // If the interpolation type is biological motion
        // Compute the trajectory for the point
        compute_trajectory(right_arm_home_position, point_position, number_of_joints, gesture_duration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the right arm to point in the specified direction
        move_to_position_biological_motion(right_arm_client, joint_names, duration_t, gesture_duration, open_hand, hand, hand_topic, "point", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){                                 // If the interpolation type is linear interpolation
        // Move the right arm to point in the specified direction
        move_to_position(right_arm_client, joint_names, gesture_duration, open_hand, hand, hand_topic, "point", point_position);
    }
}

/*  
 *   Function to move the left arm to point in a particular direction as specified by the joint angles
 *   The function moves the left arm to point in the specified direction using the control client
 *
 *   @param:
 *       left_arm_topic: the topic for the left arm
 *       shoulder_pitch: the pitch angle of the shoulder
 *       shoulder_roll: the roll angle of the shoulder
 *       elbow_yaw: the yaw angle of the elbow
 *       elbow_roll: the roll angle of the elbow
 *       wrist_yaw: the yaw angle of the wrist
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void left_arm_pointing(std::string left_arm_topic, double shoulder_pitch, double shoulder_roll, double elbow_yaw, double elbow_roll, double wrist_yaw, double gesture_duration, int interpolation, bool debug){
    // Create a control client for the left arm
    ControlClientPtr left_arm_client = create_client(left_arm_topic);
    if(left_arm_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    int number_of_joints = joint_names.size();                                      // Set the number of joints in the left arm to the size of the joint names   

    // Point position for the left arm
    std::vector<double> point_position = {shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw};
    
    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Variables to control the hand. Necessary for the left arm, thus set to true
    bool open_hand = true;
    string hand = "LHand";
    string hand_topic = "/pepper_dcm/LeftHand_controller/follow_joint_trajectory";

    // Move the left arm to point in the specified direction
    if(interpolation == BIOLOGICAL_MOTION){                                         // If the interpolation type is biological motion
        // Compute the trajectory for the point
        compute_trajectory(left_arm_home_position, point_position, number_of_joints, gesture_duration, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the left arm to point in the specified direction
        move_to_position_biological_motion(left_arm_client, joint_names, duration_t, gesture_duration, open_hand, hand, hand_topic, "side", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){                                 // If the interpolation type is linear interpolation
        // Move the left arm to point in the specified direction
        move_to_position(left_arm_client, joint_names, gesture_duration, open_hand, hand, hand_topic, "side", point_position);
    }
}

/*  
 *   Function to move the head in a nodding motion as specified by the nod angle
 *   
 *   @param:
 *       head_topic: the topic for the head
 *       nod_angle: the angle of the nod
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void head_nodding(std::string head_topic, int nod_angle, int gesture_duration, int interpolation, bool debug) {
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    if(head_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"HeadPitch", "HeadYaw"};                // Set the joint names for the head to the specified joint names
    std::vector<double> nod_position;                                               // Vector to store the nod position
    int number_of_joints = joint_names.size();                                      // Set the number of joints in the head to the size of the joint names

    // Compute the set angle for the nod
    double max_angle = double(MAX_NOD_ANGLE) * 1 * M_PI / 180;
    double min_angle = double(MIN_NOD_ANGLE) * 1 * M_PI / 180;
    double set_angle = double(nod_angle) * 1 * M_PI / 180;
    
    // Compute the duration of the gesture
    double duration_value = double(gesture_duration) / 1000;
    duration_value = 0.5;

    // Check if the nod angle is within the range
    if(set_angle > max_angle){
        if(debug){
            ROS_WARN("%s: the nod angle is greater than the maximum nod angle. Setting the nod angle to the maximum angle", node_name.c_str());
        }
        set_angle = max_angle;
    }
    else if(set_angle < min_angle){
        if(debug){
            ROS_WARN("%s: the nod angle is less than the minimum angle. Setting the nod angle to the minimum angle", node_name.c_str());
        }
        set_angle = min_angle;
    }

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Set the nod position
    nod_position = {-0.2, 0.012271};
    nod_position[0] = set_angle;

    // Variables to control the hand, not necessary for the head, thus kept at false
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Move the head in a nodding motion
    if(interpolation == BIOLOGICAL_MOTION){                                         // If the interpolation type is biological motion
        // Compute the trajectory for the nod
        compute_trajectory(head_home_position, nod_position, number_of_joints, duration_value, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the head
        move_to_position_biological_motion(head_client, joint_names, duration_t, gesture_duration, open_hand, hand, hand_topic, "nod", positions_t, velocities_t, accelerations_t);
    }
    else if(interpolation == LINEAR_INTERPOLATION){                                 // If the interpolation type is linear interpolation
        // Move the head in a nodding motion
        move_to_position(head_client, joint_names, duration_value, open_hand, hand, hand_topic, "nod", nod_position);
    }
}

/*  
 *   Function to move the leg in a bowing motion as specified by the bow angle
 *
 *   @param:
 *       leg_topic: the topic for the leg
 *       bow_angle: the angle of the bow
 *       gesture_duration: the duration of the gesture
 *       interpolation: the type of interpolation to use
 *       debug: boolean to indicate if debugging information should be printed
 *
 *   @return:
 *       None
 */
void leg_bowing(std::string leg_topic, int bow_angle, int gesture_duration, int interpolation, bool debug){
    // Create a control client for the leg
    ControlClientPtr leg_client = create_client(leg_topic);
    if(leg_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"HipPitch", "HipRoll", "KneePitch"};    // Set the joint names for the leg to the specified joint names
    std::vector<double> bow_position;                                               // Vector to store the bow position
    int number_of_joints = joint_names.size();                                      // Set the number of joints in the leg to the size of the joint names

    // Compute the set angle for the bow
    double min_angle = double(MIN_BOW_ANGLE) * M_PI * 1 / 180;
    double max_angle = double(MAX_BOW_ANGLE) * M_PI * 1 / 180;
    double set_angle = double(bow_angle) * M_PI * -1 / 180;

    // Check if the nod angle is within the range
    if(set_angle > max_angle){
        if(debug){
            ROS_WARN("%s: the bow angle is greater than the maximum nod angle. Setting the bow angle to the maximum angle", node_name.c_str());
        }
        set_angle = max_angle;
    }
    else if(set_angle < min_angle){
        if(debug){
            ROS_WARN("%s: the bow angle is less than the minimum angle. Setting the bow angle to the minimum angle", node_name.c_str());
        }
        set_angle = min_angle;
    }

    // Compute the duration of the gesture
    double duration_value = double(gesture_duration) / 1000;

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

    // Variables to control the hand, not necessary for the leg, thus kept at false
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";  

    // Set the bow position
    bow_position = {-0.0107, -0.00766, 0.03221};
    bow_position[0] = set_angle;

    // Move the leg in a bowing motion
    if(interpolation == BIOLOGICAL_MOTION){                                         // If the interpolation type is biological motion
        // Compute the trajectory for the bow
        compute_trajectory(leg_home_position, bow_position, number_of_joints, duration_value, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the leg in a bowing motion
        move_to_position_biological_motion(leg_client, joint_names, duration_t, gesture_duration, open_hand, hand, hand_topic, "bow", positions_t, velocities_t, accelerations_t);
        
    }
    else if(interpolation == LINEAR_INTERPOLATION){                                 // If the interpolation type is linear interpolation
        // Move the leg in a bowing motion
        move_to_position(leg_client, joint_names, duration_value, open_hand, hand, hand_topic, "bow", bow_position);
    }
}

/*  
 *   Function to move the right hand to open. close or home position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_topic: the topic for the hand
 *       state: the state of the hand (open, close or home)
 *   
 *   @return:
 *       None
 */
void right_hand_control(std::string right_hand_topic, std::string state){
    // Create a control client for the right hand
    ControlClientPtr right_hand_client = create_client(right_hand_topic);
    if(right_hand_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"RHand"};                               // Set the joint names for the hand to the specified hand
    
    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};
     
    // Move the hand to the specified position
    if(state == "open"){                                                            // If the state is open, send a goal to open the hand
        move_hand_to_position(right_hand_client, joint_names, 0.25, "open", open_position);
    }
    else if(state == "close"){                                                      // If the state is close, send a goal to close the hand
        move_hand_to_position(right_hand_client, joint_names, 0.25, "close", closed_position);
    }
    else if(state == "home"){                                                       // If the state is home, send a goal to take the hand to the home position
        move_hand_to_position(right_hand_client, joint_names, 0.25, "home", home_position);
    }
}

/*  
 *   Function to move the left hand to open. close or home position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_topic: the topic for the hand
 *       state: the state of the hand (open, close or home)
 *   
 *   @return:
 *       None
 */
void left_hand_control(std::string left_hand_topic, std::string state){
    // Create a control client for the left hand
    ControlClientPtr left_hand_client = create_client(left_hand_topic);
    if(left_hand_client == NULL){
        return;
    }
    std::vector<std::string> joint_names = {"LHand"};                               // Set the joint names for the hand to the specified hand
    
    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.6695};

    // Move the hand to the specified position
    if(state == "open"){                                                            // If the state is open, send a goal to open the hand
        move_hand_to_position(left_hand_client, joint_names, 0.5, "open", open_position);
    }
    else if(state == "close"){                                                      // If the state is close, send a goal to close the hand
        move_hand_to_position(left_hand_client, joint_names, 0.5, "close", closed_position);
    }
    else if(state == "home"){                                                       // If the state is home, send a goal to take the hand to the home position
        move_hand_to_position(left_hand_client, joint_names, 0.5, "home", home_position);
    }
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
    double angle_radians;                                                           // stores the angle in radians
    angle_radians = radians(angle_degrees);                                         // Convert angle from degrees to radians (function found in pepperKinematicsUtilities.h)

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
    ros::Duration(3.0).sleep();
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
    double time_t = 0;                                                              // stores the instantaneous time of the trajectory
    std::vector<double> positions_t;                                                // vector to store the positions of the trajectory
    std::vector<double> velocities_t;                                               // vector to store the velocities of the trajectory
    std::vector<double> accelerations_t;                                            // vector to store the accelerations of the trajectory
    std::vector<double> duration_t;                                                 // vector to store the duration of the trajectory
    double acceleration;                                                            // stores the acceleration
    double velocity;                                                                // stores the velocity
    double position;                                                                // stores the position
    double time_step = 1;                                                           // Time step between each point in the trajectory

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
        for(int i = 0; i < number_of_joints; i++){                                  // Create a trajectory for each joint (5 joints for the arm)
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
    // Compute the trajectory for the last point in time
    time_t = trajectory_duration;
    for(int i = 0; i < number_of_joints; i++){                                      // Create a trajectory for each joint (5 joints for the arm)   
        position = start_position[i] + (end_position[i] - start_position[i]) * ((10 * (pow(time_t/trajectory_duration, 3))) - (15 * (pow(time_t/trajectory_duration, 4))) + (6 * (pow(time_t/trajectory_duration, 5))));
        positions_t.push_back(position);

        velocity = ((end_position[i] - start_position[i])/trajectory_duration) * ((30 * (pow(time_t/trajectory_duration, 2))) - (60 * (pow(time_t/trajectory_duration, 3))) + (30 * (pow(time_t/trajectory_duration, 4))));
        velocities_t.push_back(velocity);

        acceleration = ((end_position[i] - start_position[i])/(trajectory_duration*trajectory_duration)) * ((60 * (pow(time_t/trajectory_duration, 1))) - (180 * (pow(time_t/trajectory_duration, 2))) + (120 * (pow(time_t/trajectory_duration, 3))));
        accelerations_t.push_back(acceleration);
    }
    // Store the computed trajectory for the last point in time
    positions.push_back(positions_t);
    velocities.push_back(velocities_t);
    accelerations.push_back(accelerations_t);
    durations.push_back(time_t);

    return;
}

/*  
 *   Function to move the hand to a particular position
 *   The function moves the hand to the specified position using the control client
 *
 *   @param:
 *       hand_client: the control client for the hand
 *       joint_names: vector containing the joint names of the hand
 *       duration: the duration of the movement
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the hand to
 *
 *   @return:
 *       None
 */
void move_hand_to_position(ControlClientPtr& hand_client, const std::vector<std::string>& joint_names, double duration, 
                        const std::string& position_name, std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                                           // Set the joint names for the hand to the specified joint names
    trajectory.points.resize(1);                                                    // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                                     // Set the positions for the hand to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);

    // Send the goal to move the hand to the specified position
    hand_client->sendGoal(goal);
    hand_client->waitForResult(ros::Duration(duration));                            // Wait for the hand to reach the specified position

    return;
}

/*  
 *   Function to move a single arm for an iconic gesture. The arm is moved to the waypoints specified in the waypoints vector
 *   The waypoints are the joint angles for the arm at each point in time.
 *
 *   @param:
 *       gesture_arm: the gesture arm
 *       joint_names: vector to store the joint names
 *       waypoints: vector to store the waypoints
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: string to store the hand that should be open or closed
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int move_single_arm_iconic(string gesture_arm, std::vector<string> joint_names, std::vector<std::vector<double>> waypoints, bool open_hand, string hand, int gesture_duration, string topics_file, int interpolation, bool debug){
    // Declare variables
    string arm_topic;                                                               // stores the arm topic
    string hand_topic;                                                              // stores the hand topic 
    ControlClientPtr arm_client;                                                    // action client to control the arm
    double gesture_duration_per_waypoint;                                           // stores the duration per waypoint

    // Extract the arm topic
    if(extract_topic(gesture_arm, topics_file, &arm_topic)){
        return 0;
    }
    // Extract the hand topic
    if(extract_topic(hand, topics_file, &hand_topic)){
        return 0;
    }
    // Create the action client for the arm
    arm_client = create_client(arm_topic);
    if(arm_client == NULL){
        return 0;
    }
    // Calculate the duration per waypoint, converted to double
    gesture_duration_per_waypoint = gesture_duration/1000.0;
    gesture_duration_per_waypoint = gesture_duration_per_waypoint/(waypoints.size());

    //move the arm to the first waypoint
    move_to_position(arm_client, joint_names, gesture_duration_per_waypoint, open_hand, hand, hand_topic, "iconic", waypoints[0]);

    // move the arm to the remaining waypoints
    if (interpolation == BIOLOGICAL_MOTION){                                        // Biological motion interpolation
        // Vectors to store the trajectory information (positions, velocities, accelerations and durations)
        std::vector<std::vector<double>> positions_t;
        std::vector<std::vector<double>> velocities_t;
        std::vector<std::vector<double>> accelerations_t;
        std::vector<double> duration_t;
        int i = 0;                                                                  // counter variable

        for(i = 0; i < waypoints.size() - 2; i++){                                  // Iterate through the waypoints till the second to the last waypoint
            // Compute the trajectory to get to the next waypoint
            compute_trajectory(waypoints[i], waypoints[i+1], joint_names.size(), gesture_duration_per_waypoint, positions_t, velocities_t, accelerations_t, duration_t);
            // Move the arm to the next waypoint
            move_to_position_biological_motion(arm_client, joint_names, duration_t, gesture_duration, open_hand, hand, hand_topic, "iconic", positions_t, velocities_t, accelerations_t);

        }
        // Compute the trajectory to get to the last waypoint
        compute_trajectory(waypoints[i], waypoints[i+1], joint_names.size(), gesture_duration_per_waypoint, positions_t, velocities_t, accelerations_t, duration_t);
        // Move the arm to the last waypoint
        move_to_position_biological_motion(arm_client, joint_names, duration_t, gesture_duration, false, hand, hand_topic, "iconic", positions_t, velocities_t, accelerations_t);
    }
    else{
        // move the arm to the remaining waypoints
        for (int i = 1; i < waypoints.size() - 1; i++){                             // Iterate through the waypoints till the second to the last waypoint
            // Move the arm to the next waypoint
            move_to_position(arm_client, joint_names, gesture_duration_per_waypoint, open_hand, hand, hand_topic, "iconic", waypoints[i]);
        }
        // Move the arm to the last waypoint
        move_to_position(arm_client, joint_names, gesture_duration_per_waypoint, false, hand, hand_topic, "iconic", waypoints[waypoints.size() - 1]);
    }

    return 1;

}

/*  
 *   Function to move both arms for an iconic gesture. The arms are moved to the waypoints specified in the waypoints vector
 *   The waypoints are the joint angles for the arms at each point in time.
 *
 *   @param:
 *       gesture_arm_1: the gesture arm 1
 *       gesture_arm_2: the gesture arm 2
 *       joint_names_arm_1: vector to store the joint names of arm 1
 *       joint_names_arm_2: vector to store the joint names of arm 2
 *       waypoints_arm_1: vector to store the waypoints of arm 1
 *       waypoints_arm_2: vector to store the waypoints of arm 2
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int move_both_arms_iconic(string gesture_arm_1, string gesture_arm_2, 
                            std::vector<string> joint_names_arm_1, std::vector<string> joint_names_arm_2, 
                            std::vector<std::vector<double>> waypoints_arm_1, std::vector<std::vector<double>> waypoints_arm_2, 
                            bool open_right_hand, bool open_left_hand, int gesture_duration, 
                            string topics_file, int interpolation, bool debug){
    // Declare variables
    string arm_topic_1;                                                             // stores the arm topic 1
    string arm_topic_2;                                                             // stores the arm topic 2
    ControlClientPtr arm_client_1;                                                  // action client to control the arm 1
    ControlClientPtr arm_client_2;                                                  // action client to control the arm 2
    double gesture_duration_per_waypoint;                                           // stores the duration per waypoint
    double gesture_duration_per_waypoint_arm_1;                                     // stores the duration per waypoint for arm 1
    double gesture_duration_per_waypoint_arm_2;                                     // stores the duration per waypoint for arm 2
    int number_of_waypoints;                                                        // stores the number of waypoints
    int number_of_waypoints_arm_1;                                                  // stores the number of waypoints for arm 1
    int number_of_waypoints_arm_2;                                                  // stores the number of waypoints for arm 2

    // Extract the arm topics
    if(extract_topic(gesture_arm_1, topics_file, &arm_topic_1)){
        return 0;
    }

     if(extract_topic(gesture_arm_2, topics_file, &arm_topic_2)){
        return 0;
    }
    
    // Create the action clients for the arms
    arm_client_1 = create_client(arm_topic_1);
    arm_client_2 = create_client(arm_topic_2);
    if((arm_client_1 == NULL) || (arm_client_2 == NULL)){
        return 0;
    }
    
    // Calculate the duration per waypoint, converted to double
    gesture_duration_per_waypoint = gesture_duration/1000.0;
    gesture_duration_per_waypoint_arm_1 = gesture_duration_per_waypoint/(waypoints_arm_1.size());
    gesture_duration_per_waypoint_arm_2 = gesture_duration_per_waypoint/(waypoints_arm_2.size());
    
    // Determine the number of waypoints
    number_of_waypoints = waypoints_arm_2.size();
    number_of_waypoints_arm_1 = waypoints_arm_1.size();
    number_of_waypoints_arm_2 = waypoints_arm_2.size();
    if (number_of_waypoints_arm_1 >= number_of_waypoints_arm_2){
        number_of_waypoints = number_of_waypoints_arm_1;
    }

    //move the arm to the first waypoint
    move_to_position_iconic(arm_client_1, arm_client_2, open_right_hand, open_left_hand, joint_names_arm_1, joint_names_arm_2, gesture_duration_per_waypoint_arm_1, gesture_duration_per_waypoint_arm_2, "iconic", topics_file, waypoints_arm_1[0], waypoints_arm_2[0]);

    // move the arm to the remaining waypoints
    if (interpolation == BIOLOGICAL_MOTION){                                        // Biological motion interpolation
        // Vectors to store the trajectory information (positions, velocities, accelerations and durations) for arm 1
        std::vector<std::vector<double>> arm_1_positions_t;
        std::vector<std::vector<double>> arm_1_velocities_t;
        std::vector<std::vector<double>> arm_1_accelerations_t;
        std::vector<double> arm_1_duration_t;

        // Vectors to store the trajectory information (positions, velocities, accelerations and durations) for arm 2
        std::vector<std::vector<double>> arm_2_positions_t;
        std::vector<std::vector<double>> arm_2_velocities_t;
        std::vector<std::vector<double>> arm_2_accelerations_t;
        std::vector<double> arm_2_duration_t;
        
        // counter variables
        int i = 0;
        int j = 0;
        int k = 0;

        // Move the arms to the remaining waypoints
        for(k = 0; k < number_of_waypoints - 2; k++){                               // Iterate through the waypoints till the second to the last waypoint
            if(i < number_of_waypoints_arm_1 - 2){                                  // If there are more waypoints for arm 1       
                // Compute the trajectory to get to the next waypoint for arm 1
                compute_trajectory(waypoints_arm_1[i], waypoints_arm_1[i+1], joint_names_arm_1.size(), gesture_duration_per_waypoint_arm_1, arm_1_positions_t, arm_1_velocities_t, arm_1_accelerations_t, arm_1_duration_t);
                i++;
            }
            if(j < number_of_waypoints_arm_2 - 2){                                  // If there are more waypoints for arm 2
                // Compute the trajectory to get to the next waypoint for arm 2
                compute_trajectory(waypoints_arm_2[j], waypoints_arm_2[j+1], joint_names_arm_2.size(), gesture_duration_per_waypoint_arm_2, arm_2_positions_t, arm_2_velocities_t, arm_2_accelerations_t, arm_2_duration_t);
                j++;
            }
            // Move the arms to the next waypoints
            move_to_position_biological_motion_iconic(arm_client_1, arm_client_2, open_right_hand, open_left_hand, joint_names_arm_1, joint_names_arm_2, arm_1_duration_t, arm_2_duration_t, gesture_duration, "iconic", topics_file, arm_1_positions_t, arm_2_positions_t, arm_1_velocities_t, arm_2_velocities_t, arm_1_accelerations_t, arm_2_accelerations_t);

            // Clear the vectors for the next iteration
            arm_1_positions_t.clear();
            arm_1_velocities_t.clear();
            arm_1_accelerations_t.clear();
            arm_1_duration_t.clear();

            arm_2_positions_t.clear();
            arm_2_velocities_t.clear();
            arm_2_accelerations_t.clear();
            arm_2_duration_t.clear();
        }
        // Compute the trajectory to get to the last waypoint for arm 1
        compute_trajectory(waypoints_arm_1[i], waypoints_arm_1[i+1], joint_names_arm_1.size(), gesture_duration_per_waypoint_arm_1, arm_1_positions_t, arm_1_velocities_t, arm_1_accelerations_t, arm_1_duration_t);
        // Compute the trajectory to get to the last waypoint for arm 2
        compute_trajectory(waypoints_arm_2[j], waypoints_arm_2[j+1], joint_names_arm_2.size(), gesture_duration_per_waypoint_arm_2, arm_2_positions_t, arm_2_velocities_t, arm_2_accelerations_t, arm_2_duration_t);
        // Move the arms to the last waypoints
        move_to_position_biological_motion_iconic(arm_client_1, arm_client_2, false, false, joint_names_arm_1, joint_names_arm_2, arm_1_duration_t, arm_2_duration_t, gesture_duration, "iconic", topics_file, arm_1_positions_t, arm_2_positions_t, arm_1_velocities_t, arm_2_velocities_t, arm_1_accelerations_t, arm_2_accelerations_t);
    }
    else{                                                                           // Linear interpolation
        // Counter variables
        int i = 1;
        int j = 1; 
        int k = 1;

        std::vector<double> waypoint_arm_1;                                         // Vector to store the waypoints for arm 1
        std::vector<double> waypoint_arm_2;                                         // Vector to store the waypoints for arm 2

        for(k = 1; k < number_of_waypoints - 1; k++){                               // Iterate through the waypoints till the second to the last waypoint
            if(i < number_of_waypoints_arm_1 - 1){                                  // If there are more waypoints for arm 1
                waypoint_arm_1 = waypoints_arm_1[i];                                // Get the next waypoint for arm 1
                i++;
            }
            if(j < number_of_waypoints_arm_2 - 1){                                  // If there are more waypoints for arm 2
                waypoint_arm_2 = waypoints_arm_2[j];                                // Get the next waypoint for arm 2
                j++;
            }
            // Move the arms to the next waypoints
            move_to_position_iconic(arm_client_1, arm_client_2, false, false, joint_names_arm_1, joint_names_arm_2, gesture_duration_per_waypoint_arm_1, gesture_duration_per_waypoint_arm_1, "iconic", topics_file, waypoint_arm_1, waypoint_arm_2);
            // Clear the vectors for the next iteration
            waypoint_arm_1.clear();
            waypoint_arm_2.clear();
        }
        // Move the arms to the last waypoints
        move_to_position_iconic(arm_client_1, arm_client_2, false, false, joint_names_arm_1, joint_names_arm_2, gesture_duration_per_waypoint_arm_1, gesture_duration_per_waypoint_arm_1, "iconic", topics_file, waypoints_arm_1[i], waypoints_arm_2[j]);
    }
    return 1;
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
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                                           // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                                    // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                                     // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);                 // Set the time from start of the trajectory to the specified duration

    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the hand
    ControlClientPtr hand_client;

    // Goal messages for the hand
    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;

    // If the hand should be opened, create a client and send a goal to open the hand
    if(open_hand){
        hand_client = create_client(hand_topic);                                    // Create a client for the hand
        if(hand_client == NULL){
            return;
        }
        std::vector<std::string> hand_joint_names = {hand};                         // Set the joint names for the hand to the specified hand

        // Set the trajectory for the hand to open
        trajectory_msgs::JointTrajectory& hand_open_trajectory = hand_open_goal.trajectory;
        hand_open_trajectory.joint_names = hand_joint_names;
        hand_open_trajectory.points.resize(1);
        hand_open_trajectory.points[0].positions = open_position;
        hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);

        // Send the goal to open the hand
        hand_client->sendGoal(hand_open_goal);                                      // Open the hand
    }

    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration));                                 // Wait for the actuator to reach the specified position

    // If the hand should not be opened, create a client and send a goal to take the hand to the home position
    if(!open_hand){
        hand_client = create_client(hand_topic);                                    // Create a client for the hand
        if(hand_client == NULL){
            return;
        }
        std::vector<std::string> hand_joint_names = {hand};                         // Set the joint names for the hand to the specified hand

        // Set the trajectory for the hand to move to home position
        trajectory_msgs::JointTrajectory& hand_close_trajectory = hand_close_goal.trajectory;
        hand_close_trajectory.joint_names = hand_joint_names;
        hand_close_trajectory.points.resize(1);
        hand_close_trajectory.points[0].positions = home_position;
        hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);

        // Send the goal to take the hand to the home position
        hand_client->sendGoal(hand_close_goal);                                     // Close the hand
    }

    return;
}

/*  Function to move the arm to a position using the minimum-jerk model of biological motion
 *   The function moves the arm to the specified position using the control client
 *
 *   @param:
 *       client: the control client for the arm
 *       joint_names: vector containing the joint names of the arm
 *       duration: vector containing the duration of the movement
 *       gesture_duration: the duration of the gesture
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the arm to
 *       velocities: vector containing the joint velocities of the position to move the arm to
 *       accelerations: vector containing the joint accelerations of the position to move the arm to
 *
 *   @return:
 *       None
 */
void move_to_position_biological_motion(ControlClientPtr& client, const std::vector<std::string>& joint_names, std::vector<double> duration, double gesture_duration,
                        bool open_hand, string hand, string hand_topic, const std::string& position_name, 
                        std::vector<std::vector<double>> positions, std::vector<std::vector<double>> velocities, std::vector<std::vector<double>> accelerations){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                                           // Set the joint names for the arm to the specified joint names
    trajectory.points.resize(positions.size());                                     // Set the number of points in the trajectory to the number of positions in the waypoints

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < positions.size(); i++){
        trajectory.points[i].positions = positions[i];
        trajectory.points[i].velocities = velocities[i];
        trajectory.points[i].accelerations = accelerations[i];
        trajectory.points[i].time_from_start = ros::Duration(duration[i]);
    }

    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the hand
    ControlClientPtr hand_client;
    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;

    // If the hand should be opened, create a client and send a goal to open the hand
    if(open_hand){
        hand_client = create_client(hand_topic);                                    // Create a client for the hand
        if(hand_client == NULL){
            return;
        }
        std::vector<std::string> hand_joint_names = {hand};                         // Set the joint names for the hand to the specified hand

        // Set the trajectory for the hand to open
        trajectory_msgs::JointTrajectory& hand_open_trajectory = hand_open_goal.trajectory;
        hand_open_trajectory.joint_names = hand_joint_names;
        hand_open_trajectory.points.resize(1);
        hand_open_trajectory.points[0].positions = open_position;
        hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);

        // Send the goal to open the hand
        hand_client->sendGoal(hand_open_goal);                                      // Open the hand
    }

    // Send the goal to move the arm to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(gesture_duration));                         // Wait for the arm to reach the specified position

    // If the hand should not be opened, create a client and send a goal to take the hand to the home position
    if(!open_hand){
        hand_client = create_client(hand_topic);                                    // Create a client for the hand
        if(hand_client == NULL){
            return;
        }
        std::vector<std::string> hand_joint_names = {hand};                         // Set the joint names for the hand to the specified hand

        // Set the trajectory for the hand to move to home position
        trajectory_msgs::JointTrajectory& hand_close_trajectory = hand_close_goal.trajectory;
        hand_close_trajectory.joint_names = hand_joint_names;
        hand_close_trajectory.points.resize(1);
        hand_close_trajectory.points[0].positions = home_position;
        hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);
        
        // Send the goal to take the hand to the home position
        hand_client->sendGoal(hand_close_goal);                                     // Close the hand
    }
    return;
}

/*  
 *   Function to move both arms to the specified positions using linear interpolation
 *   The function moves both arms to the specified positions using the control clients
 *
 *   @param:
 *       arm_1_client: the control client for the first arm
 *       arm_2_client: the control client for the second arm
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       arm_1_joint_names: vector containing the joint names of the first arm
 *       arm_2_joint_names: vector containing the joint names of the second arm
 *       arm_1_duration: the duration of the movement for the first arm
 *       arm_2_duration: the duration of the movement for the second arm
 *       position_name: the name of the position
 *       arm_1_positions: vector containing the joint angles of the first arm
 *       arm_2_positions: vector containing the joint angles of the second arm
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int move_to_position_iconic(ControlClientPtr& arm_1_client, ControlClientPtr& arm_2_client, bool open_right_hand, bool open_left_hand, 
                        const std::vector<std::string>& arm_1_joint_names, const std::vector<std::string>& arm_2_joint_names, 
                        double arm_1_duration, double arm_2_duration, const std::string& position_name, string topics_file,
                        std::vector<double> arm_1_positions, std::vector<double> arm_2_positions){
    // Create goal message for the first arm
    control_msgs::FollowJointTrajectoryGoal arm_1_goal;
    trajectory_msgs::JointTrajectory& arm_1_trajectory = arm_1_goal.trajectory;
    arm_1_trajectory.joint_names = arm_1_joint_names;                               // Set the joint names for the first arm to the specified joint names
    arm_1_trajectory.points.resize(1);                                              // Set the number of points in the trajectory to 1

    arm_1_trajectory.points[0].positions = arm_1_positions;                         // Set the positions in the trajectory to the specified positions for arm 1
    arm_1_trajectory.points[0].time_from_start = ros::Duration(arm_1_duration);

    // Create goal message for the second arm
    control_msgs::FollowJointTrajectoryGoal arm_2_goal;
    trajectory_msgs::JointTrajectory& arm_2_trajectory = arm_2_goal.trajectory;
    arm_2_trajectory.joint_names = arm_2_joint_names;                               // Set the joint names for the second arm to the specified joint names
    arm_2_trajectory.points.resize(1);                                              // Set the number of points in the trajectory to 1

    arm_2_trajectory.points[0].positions = arm_2_positions;                         // Set the positions in the trajectory to the specified positions for arm 2
    arm_2_trajectory.points[0].time_from_start = ros::Duration(arm_2_duration);

    // Joint angles of the hand positions
    std::vector<double> open_position = {0.9};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the right hand
    ControlClientPtr right_hand_client;

    // Goal messages for the right hand
    control_msgs::FollowJointTrajectoryGoal right_hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal right_hand_close_goal;
    
    // control client for the left hand
    ControlClientPtr left_hand_client;

    // Goal messages for the left hand
    control_msgs::FollowJointTrajectoryGoal left_hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal left_hand_close_goal;

    // Topic names for the hands
    std::string right_hand_topic;
    std::string left_hand_topic;
    
    if(extract_topic("RHand", topics_file, &right_hand_topic)){
        return 0;
    }

    if(extract_topic("LHand", topics_file, &left_hand_topic)){
        return 0;
    }
    
    // If the right hand should be opened, create a client and send a goal to open the right hand
    if(open_right_hand){
        right_hand_client = create_client(right_hand_topic);                        // Create a client for the right hand
        if(right_hand_client == NULL){
            return 0;
        }
        // right_hand_client = create_client("/pepper_dcm/RightHand_controller/follow_joint_trajectory");
        std::vector<std::string> right_hand_joint_names = {"RHand"};                // Set the joint names for the right hand to the specified hand

        // Set the trajectory for the right hand to open
        trajectory_msgs::JointTrajectory& right_hand_open_trajectory = right_hand_open_goal.trajectory;
        right_hand_open_trajectory.joint_names = right_hand_joint_names;
        right_hand_open_trajectory.points.resize(1);
        right_hand_open_trajectory.points[0].positions = open_position;
        right_hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // If the left hand should be opened, create a client and send a goal to open the left hand
    if(open_left_hand){
        left_hand_client = create_client(left_hand_topic);                          // Create a client for the left hand
        if(left_hand_client == NULL){
            return 0;
        }
        // left_hand_client = create_client("/pepper_dcm/LeftHand_controller/follow_joint_trajectory");
        std::vector<std::string> left_hand_joint_names = {"LHand"};                 // Set the joint names for the left hand to the specified hand

        // Set the trajectory for the left hand to open
        trajectory_msgs::JointTrajectory& left_hand_open_trajectory = left_hand_open_goal.trajectory;
        left_hand_open_trajectory.joint_names = left_hand_joint_names;
        left_hand_open_trajectory.points.resize(1);
        left_hand_open_trajectory.points[0].positions = open_position;
        left_hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // If the positions for the first arm are not empty, send a goal to move the first arm to the specified position
    if(!arm_1_positions.empty()){
        if(open_right_hand){                                                        // If the right hand should be opened, send a goal to open the right hand
            right_hand_client->sendGoal(right_hand_open_goal);                      // Open the right hand
        }
        arm_1_client->sendGoal(arm_1_goal);
    }

    // If the positions for the second arm are not empty, send a goal to move the second arm to the specified position
    if(!arm_2_positions.empty()){
        if(open_left_hand){                                                         // If the left hand should be opened, send a goal to open the left hand
            left_hand_client->sendGoal(left_hand_open_goal);                        // Open the left hand
        }
        arm_2_client->sendGoal(arm_2_goal);
    }

    // Set the wait time to the maximum of the duration of the first and second arms
    double wait_time;
    wait_time = arm_1_duration > arm_2_duration ? arm_1_duration : arm_2_duration;
    arm_1_client->waitForResult(ros::Duration(wait_time));                          // Adjust the timeout as needed
    // arm_2_client->waitForResult(ros::Duration(duration));                           // Adjust the timeout as needed

    // If the right hand should not be opened, create a client and send a goal to take the right hand to the home position
    if(!open_right_hand){
        right_hand_client = create_client(right_hand_topic);                        // Create a client for the right hand
        if(right_hand_client == NULL){
            return 0;
        }
        // right_hand_client = create_client("/pepper_dcm/RightHand_controller/follow_joint_trajectory");
        std::vector<std::string> right_hand_joint_names = {"RHand"};                // Set the joint names for the right hand to the specified hand

        // Set the trajectory for the right hand to move to home position
        trajectory_msgs::JointTrajectory& right_hand_close_trajectory = right_hand_close_goal.trajectory;
        right_hand_close_trajectory.joint_names = right_hand_joint_names;
        right_hand_close_trajectory.points.resize(1);
        right_hand_close_trajectory.points[0].positions = home_position;
        right_hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // If the left hand should not be opened, create a client and send a goal to take the left hand to the home position
    if(!open_left_hand){
        left_hand_client = create_client(left_hand_topic);                          // Create a client for the left hand
        if(left_hand_client == NULL){
            return 0;
        }
        // left_hand_client = create_client("/pepper_dcm/LeftHand_controller/follow_joint_trajectory");
        std::vector<std::string> left_hand_joint_names = {"LHand"};                 // Set the joint names for the left hand to the specified hand

        // Set the trajectory for the left hand to move to home position
        trajectory_msgs::JointTrajectory& left_hand_close_trajectory = left_hand_close_goal.trajectory;
        left_hand_close_trajectory.joint_names = left_hand_joint_names;
        left_hand_close_trajectory.points.resize(1);
        left_hand_close_trajectory.points[0].positions = home_position;
        left_hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // If the right hand should not be opened, send a goal to take the right hand to the home position
    if(!open_right_hand){
        right_hand_client->sendGoal(right_hand_close_goal);                         // Close the right hand
    }
    // If the left hand should not be opened, send a goal to take the left hand to the home position
    if(!open_left_hand){
        left_hand_client->sendGoal(left_hand_close_goal);                           // Close the left hand
    }

    return 1;
}

/*  
 *   Function to move both arms to the specified positions using the minimum-jerk model of biological motion
 *   The function moves both arms to the specified positions in their respective waypoints using the control clients
 *
 *   @param:
 *       arm_1_client: the control client for the first arm
 *       arm_2_client: the control client for the second arm
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       arm_1_joint_names: vector containing the joint names of the first arm
 *       arm_2_joint_names: vector containing the joint names of the second arm
 *       arm_1_duration: vector containing the duration of the movement for the first arm
 *       arm_2_duration: vector containing the duration of the movement for the second arm
 *       gesture_duration: the duration of the gesture
 *       position_name: the name of the position
 *       arm_1_positions: vector containing the joint angles of the first arm
 *       arm_2_positions: vector containing the joint angles of the second arm
 *       arm_1_velocities: vector containing the joint velocities of the first arm
 *       arm_2_velocities: vector containing the joint velocities of the second arm
 *       arm_1_accelerations: vector containing the joint accelerations of the first arm
 *       arm_2_accelerations: vector containing the joint accelerations of the second arm
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int move_to_position_biological_motion_iconic(ControlClientPtr& arm_1_client, ControlClientPtr& arm_2_client, bool open_right_hand, bool open_left_hand, 
                        const std::vector<std::string>& arm_1_joint_names, const std::vector<std::string>& arm_2_joint_names, 
                        std::vector<double> arm_1_duration, std::vector<double> arm_2_duration, double gesture_duration, 
                        const std::string& position_name, string topics_file,
                        std::vector<std::vector<double>> arm_1_positions, std::vector<std::vector<double>> arm_2_positions, 
                        std::vector<std::vector<double>> arm_1_velocities, std::vector<std::vector<double>> arm_2_velocities, 
                        std::vector<std::vector<double>> arm_1_accelerations, std::vector<std::vector<double>> arm_2_accelerations){
    // Create goal message for the first arm
    control_msgs::FollowJointTrajectoryGoal arm_1_goal;
    trajectory_msgs::JointTrajectory& arm_1_trajectory = arm_1_goal.trajectory;
    arm_1_trajectory.joint_names = arm_1_joint_names;
    arm_1_trajectory.points.resize(arm_1_positions.size());

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < arm_1_positions.size(); i++){
        arm_1_trajectory.points[i].positions = arm_1_positions[i];
        arm_1_trajectory.points[i].velocities = arm_1_velocities[i];
        arm_1_trajectory.points[i].accelerations = arm_1_accelerations[i];
        arm_1_trajectory.points[i].time_from_start = ros::Duration(arm_1_duration[i]);
    }

    // Create goal message for the second arm
    control_msgs::FollowJointTrajectoryGoal arm_2_goal;
    trajectory_msgs::JointTrajectory& arm_2_trajectory = arm_2_goal.trajectory;
    arm_2_trajectory.joint_names = arm_2_joint_names;
    arm_2_trajectory.points.resize(arm_2_positions.size());

    // Set the positions, velocities, accelerations and time from start for each point in the trajectory
    for(int i = 0; i < arm_2_positions.size(); i++){
        arm_2_trajectory.points[i].positions = arm_2_positions[i];
        arm_2_trajectory.points[i].velocities = arm_2_velocities[i];
        arm_2_trajectory.points[i].accelerations = arm_2_accelerations[i];
        arm_2_trajectory.points[i].time_from_start = ros::Duration(arm_2_duration[i]);
    }

    // Joint angles of the hand positions
    std::vector<double> open_position = {0.8};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the right hand
    ControlClientPtr right_hand_client;
    control_msgs::FollowJointTrajectoryGoal right_hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal right_hand_close_goal;
    
    // control client for the left hand
    ControlClientPtr left_hand_client;
    control_msgs::FollowJointTrajectoryGoal left_hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal left_hand_close_goal;

    // Topic names for the hands
    std::string right_hand_topic;
    std::string left_hand_topic;
    
    if(extract_topic("RHand", topics_file, &right_hand_topic)){
        return 0;
    }

    if(extract_topic("LHand", topics_file, &left_hand_topic)){
        return 0;
    }

    // If the right hand should be opened, create a client and send a goal to open the right hand
    if(open_right_hand){
        right_hand_client = create_client(right_hand_topic);                        // Create a client for the right hand
        if(right_hand_client == NULL){
            return 0;
        }
        // right_hand_client = create_client("/pepper_dcm/RightHand_controller/follow_joint_trajectory");
        std::vector<std::string> right_hand_joint_names = {"RHand"};                // Set the joint names for the right hand to the specified hand
        
        // Set the trajectory for the right hand to open
        trajectory_msgs::JointTrajectory& right_hand_open_trajectory = right_hand_open_goal.trajectory;
        right_hand_open_trajectory.joint_names = right_hand_joint_names;
        right_hand_open_trajectory.points.resize(1);
        right_hand_open_trajectory.points[0].positions = open_position;
        right_hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // If the left hand should be opened, create a client and send a goal to open the left hand 
    if(open_left_hand){
        left_hand_client = create_client(left_hand_topic);                          // Create a client for the left hand
        if(left_hand_client == NULL){
            return 0;
        }
        std::vector<std::string> left_hand_joint_names = {"LHand"};                 // Set the joint names for the left hand to the specified hand
        
        // Set the trajectory for the left hand to open
        trajectory_msgs::JointTrajectory& left_hand_open_trajectory = left_hand_open_goal.trajectory;
        left_hand_open_trajectory.joint_names = left_hand_joint_names;
        left_hand_open_trajectory.points.resize(1);
        left_hand_open_trajectory.points[0].positions = open_position;
        left_hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // Send a goal to move the first arm to the specified position if the positions are not empty
    if(!arm_1_positions.empty()){
        if(open_right_hand){                                                        // If the right hand should be opened, send a goal to open the right hand
            right_hand_client->sendGoal(right_hand_open_goal);                      // Open the right hand
        }
        arm_1_client->sendGoal(arm_1_goal);
    }
    // Send a goal to move the second arm to the specified position if the positions are not empty
    if(!arm_2_positions.empty()){
        if(open_left_hand){                                                         // If the left hand should be opened, send a goal to open the left hand
            left_hand_client->sendGoal(left_hand_open_goal);                        // Open the left hand
        }
        arm_2_client->sendGoal(arm_2_goal);
    }
    
    // Wait for the arms to reach the specified positions
    if(!arm_1_positions.empty()){
        arm_1_client->waitForResult(ros::Duration(gesture_duration));               // Wait for the first arm to reach the specified position
    }
    if(!arm_2_positions.empty()){
        arm_2_client->waitForResult(ros::Duration(gesture_duration));               // Wait for the second arm to reach the specified position
    }

    // If the right hand should not be opened, create a client and send a goal to take the right hand to the home position
    if(!open_right_hand){
        right_hand_client = create_client(right_hand_topic);                        // Create a client for the right hand
        if(right_hand_client == NULL){
            return 0;
        }
        std::vector<std::string> right_hand_joint_names = {"RHand"};                // Set the joint names for the right hand to the specified hand

        // Set the trajectory for the right hand to move to home position
        trajectory_msgs::JointTrajectory& right_hand_close_trajectory = right_hand_close_goal.trajectory;
        right_hand_close_trajectory.joint_names = right_hand_joint_names;
        right_hand_close_trajectory.points.resize(1);
        right_hand_close_trajectory.points[0].positions = home_position;
        right_hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }
    // If the left hand should not be opened, create a client and send a goal to take the left hand to the home position
    if(!open_left_hand){
        left_hand_client = create_client(left_hand_topic);                          // Create a client for the left hand
        if(left_hand_client == NULL){
            return 0;
        }
        std::vector<std::string> left_hand_joint_names = {"LHand"};

        // Set the trajectory for the left hand to move to home position
        trajectory_msgs::JointTrajectory& left_hand_close_trajectory = left_hand_close_goal.trajectory;
        left_hand_close_trajectory.joint_names = left_hand_joint_names;
        left_hand_close_trajectory.points.resize(1);
        left_hand_close_trajectory.points[0].positions = home_position;
        left_hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);
    }

    // Send a goal to take the right hand to the home position if the right hand should not be opened
    if(!arm_1_positions.empty()){
        if(!open_right_hand){                                                       // If the right hand should not be opened, send a goal to take the right hand to the home position
            right_hand_client->sendGoal(right_hand_close_goal);                     // Close the right hand
        }
    }
    // Send a goal to take the left hand to the home position if the left hand should not be opened
    if(!arm_2_positions.empty()){
        if(!open_left_hand){                                                        // If the left hand should not be opened, send a goal to take the left hand to the home position
            left_hand_client->sendGoal(left_hand_close_goal);                       // Close the left hand
        }
    }

    return 1;
}





/*  --------------------------------------------------
            GESTURE CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to execute deictic gestures. The deictic gestures are executed by pointing the robot's arm to the specified point in the environment.
 *   The pointing coordinates are the x, y and z coordinates of the point in the environment.
 *
 *   @param:
 *       point_x: the x coordinate of the point
 *       point_y: the y coordinate of the point
 *       point_z: the z coordinate of the point
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       velocity_publisher: the velocity publisher
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int deictic_gesture(float point_x, float point_y, float point_z, int gesture_duration, string topics_file, int interpolation, ros::Publisher velocity_publisher, bool debug){
    bool debug_mode = false;   // used to turn debug message on
    // Robot pose coordinates
    double robot_x      = robot_pose[0] * 1000;                                     // Convert the robot's x coordinate from meters to millimeters
    double robot_y      = robot_pose[1] * 1000;                                     // Convert the robot's y coordinate from meters to millimeters
    double robot_theta  = robot_pose[2];
    robot_theta         = radians(robot_theta);                                     // Convert the robot's orientation from degrees to radians

    // Pointing coordinates
    double pointing_x    = 0.0;
    double pointing_y    = 0.0;
    double pointing_z    = 0.0;

    // Elbow coordinates
    double elbow_x       = 0.0;
    double elbow_y       = 0.0;
    double elbow_z       = 0.0;
    
    // Arm variables
    int pointing_arm    = LEFT_ARM;
    bool pose_achievable= true;
    double rotation_angle = 0.0;
    float l_1           = UPPER_ARM_LENGTH;
    float l_2           = 0.0;
    
    // Shoulder coordinates
    float x_s           = SHOULDER_OFFSET_X;
    float y_s           = SHOULDER_OFFSET_Y;
    float z_s           = SHOULDER_OFFSET_Z + TORSO_HEIGHT;

    // Head coordinates
    double head_x       = -38.0;
    double head_y       = 0.0;
    double head_z       = 169.9 + TORSO_HEIGHT;
    double l_head_1     = 112.051;
    double l_head_2     = 0.0;

    // Camera coordinates
    double camera_x     = 0.0;
    double camera_y     = 0.0;
    double camera_z     = 0.0;
    
    // Gesture duration in milliseconds
    double duration     = gesture_duration/1000.0;


    /* Compute the pointing coordinates with respect to the robot pose in the environment */
    double relative_pointing_x = (point_x * 1000) - robot_x;                        // Convert the pointing coordinates from meters to millimeters
    double relative_pointing_y = (point_y * 1000) - robot_y;                        // Convert the pointing coordinates from meters to millimeters
    pointing_x = (relative_pointing_x * cos(-robot_theta)) - (relative_pointing_y * sin(-robot_theta));
    pointing_y = (relative_pointing_y * cos(-robot_theta)) + (relative_pointing_x * sin(-robot_theta));
    pointing_z = point_z * 1000;                                                    // The pointing z coordinate is the same as the robot's z coordinate (converted to millimeters)
    
    ROS_INFO("%s: Pointing coordinates: (%f, %f, %f)", node_name.c_str(), pointing_x, pointing_y, pointing_z);
    /* Account for unreachable points in the cartesian space 
    (e.g. outside the robot's forward reach)
    Rotate the robot appropriately (by 90 degrees) if necessary */

    // Case 1: Pointing coordinates directly in front of the robot (+x direction): No rotation is needed, just choose arm
    if(pointing_x >= 0.0){
        pose_achievable = true;                                                     // The pointing coordinates are reachable without rotating the robot
        // Use right arm if the pointing coordinates are to the right of the robot (-y direction)
        if(pointing_y <= 0.0){
            pointing_arm = RIGHT_ARM;
            y_s = -y_s;
        }
        // Use left arm if the pointing coordinates are to the left of the robot (+y direction)
        else if(pointing_y > 0.0){
            pointing_arm = LEFT_ARM;
        }
    }
    // Case 2: Pointing coordinates directly behind the robot (-x direction): 
    // Rotate the robot by 90 degrees left or right depending on the y coordinate of the pointing coordinates
    else if(pointing_x < 0.0){
        pose_achievable = false;
        double temp_var = 0.0;
        // Rotate 90 degrees clockwise and use right arm if the pointing coordinates are to the right of the robot (-y direction)
        if(pointing_y <= 0.0){
            pointing_arm = RIGHT_ARM;
            rotation_angle = -90.0;
            y_s = -y_s;
            // Realign the pointing coordinates considering the rotation
            temp_var = pointing_x;
            pointing_x = -pointing_y;
            pointing_y = temp_var;
        }
        // Rotate 90 degrees anticlockwise and use left arm if the pointing coordinates are to the left of the robot (+y direction)
        else if(pointing_y > 0.0){
            pointing_arm = LEFT_ARM;
            rotation_angle = 90.0;
            // Realign the pointing coordinates considering the rotation
            temp_var = pointing_x;
            pointing_x = pointing_y;
            pointing_y = -temp_var;
        }
    }

    // Calculate the elbow coordinates
    double distance = sqrt(pow((pointing_x - x_s), 2) + pow((pointing_y - y_s), 2) + pow((pointing_z - z_s), 2));
    l_2 = distance - l_1;

    elbow_x = ((l_1 * pointing_x) + (l_2 * x_s))/(l_1 + l_2);
    elbow_y = ((l_1 * pointing_y) + (l_2 * y_s))/(l_1 + l_2);
    elbow_z = ((l_1 * pointing_z) + (l_2 * z_s))/(l_1 + l_2);
    elbow_z = elbow_z - TORSO_HEIGHT;

    // // Calculate the camera coordinates
    // distance = sqrt(pow((pointing_x - head_x), 2) + pow((pointing_y - head_y), 2) + pow((pointing_z - head_z), 2));
    // l_head_2 = distance - l_head_1;

    // camera_x = ((l_head_1 * pointing_x) + (l_head_2 * head_x))/(l_head_1 + l_head_2);
    // camera_y = ((l_head_1 * pointing_y) + (l_head_2 * head_y))/(l_head_1 + l_head_2);
    // camera_z = ((l_head_1 * pointing_z) + (l_head_2 * head_z))/(l_head_1 + l_head_2);
    // camera_z = camera_z + 61.6 - TORSO_HEIGHT;

    // Arm joint angles
    double shoulder_pitch;
    double shoulder_roll;
    double elbow_yaw;
    double elbow_roll;
    double wrist_yaw;

    // Shoulder pitch and shoulder roll limits for the pointing arm
    double shoulder_pitch_max;
    double shoulder_pitch_min;
    double shoulder_roll_max;
    double shoulder_roll_min;

    // Set the shoulder pitch and shoulder roll limits for the pointing arm
    if(pointing_arm == RIGHT_ARM){                                                  // Right arm
        shoulder_pitch_min = MIN_RSHOULDER_PITCH;
        shoulder_pitch_max = MAX_RSHOULDER_PITCH;
        shoulder_roll_min = MIN_RSHOULDER_ROLL;
        shoulder_roll_max = MAX_RSHOULDER_ROLL;
    }
    else if(pointing_arm == LEFT_ARM){                                              // Left arm
        shoulder_pitch_min = MIN_LSHOULDER_PITCH;
        shoulder_pitch_max = MAX_LSHOULDER_PITCH;
        shoulder_roll_min = MIN_LSHOULDER_ROLL;
        shoulder_roll_max = MAX_LSHOULDER_ROLL;
    }

    // Obtain the shoulder pitch and shoulder roll angles for the pointing arm. Wrist position values are set to zero since they are not used
    get_arm_angles(pointing_arm, elbow_x, elbow_y, elbow_z, 0.0, 0.0, 0.0, &shoulder_pitch, &shoulder_roll, &elbow_yaw, &elbow_roll);

    if(debug){
        ROS_INFO("%s: shoulderPitch: %f, shoulderRoll: %f", node_name.c_str(), shoulder_pitch, shoulder_roll);
    }

    // Check if the shoulder pitch value is within the limits
    if(shoulder_pitch < shoulder_pitch_min || shoulder_pitch > shoulder_pitch_max){
        if(debug){
            ROS_ERROR("%s: shoulderPitch value out of range", node_name.c_str());
        }
        return 0;                                                                   // gesture not executed successfully
    }

    // Check if the shoulder roll value is within the limits
    if(shoulder_roll < shoulder_roll_min || shoulder_roll > shoulder_roll_max){
        if(debug){
            ROS_ERROR("%s: shoulderRoll value out of range", node_name.c_str());
        }
        return 0;                                                                   // gesture not executed successfully
    }

    // Check if the shoulder pitch and shoulder roll values are valid
    if(isnan(shoulder_pitch) || isnan(shoulder_roll)){
        if(debug){
            ROS_ERROR("%s: invalid shoulderPitch or shoulderRoll value", node_name.c_str());
        }
        return 0;                                                                   // gesture not executed successfully
    }

    std::string arm_topic;                                                          // stores the arm topic
    std::string head_topic;                                                         // stores the head topic
    std::string hand_topic;                                                         // stores the hand topic

    // Rotate the robot by 90 degrees if the pointing coordinates are unreachable
    if(!pose_achievable){
        rotate_robot(rotation_angle, velocity_publisher, debug);
    }

    // Set the required fields to the overtAttention/set_mode service request
    overt_attention_srv.request.state = "location";
    overt_attention_srv.request.location_x = pointing_x / 1000;
    overt_attention_srv.request.location_y = pointing_y / 1000;
    overt_attention_srv.request.location_z = pointing_z / 1000;

    // Execute the pointing gesture
    if(pointing_arm == RIGHT_ARM){                                                  // Right arm pointing gesture
        // Extract the topic for the right arm
        if(extract_topic("RArm", topics_file, &arm_topic)){
            return 0;
        }

        // Extract the topic for the right hand
        if(extract_topic("RHand", topics_file, &hand_topic)){
            return 0;
        }

        // Set the joint angles for the right arm
        elbow_yaw = 2.0857;                                                         // Value for elbowYaw for pointing gesture
        elbow_roll = 0.0;                                                           // Value for elbowRoll for pointing gesture
        wrist_yaw = -0.05679;                                                       // Value for wristYaw for pointing gesture

        // Move the arm to the home position
        int send_arm_to_home;
        send_arm_to_home = go_to_home("RArm", topics_file, interpolation, debug); 
        if(send_arm_to_home == 0){
            ROS_ERROR("%s: error moving the right arm to the home position.", node_name.c_str());
            return 0;                                                               // gesture not executed successfully
        }

        // Call the overtAttention/set_mode service to look at the location
        overt_attention_client.call(overt_attention_srv);

        if(!overt_attention_srv.response.mode_set_success){
            ROS_ERROR("%s: error setting the mode for overt attention.", node_name.c_str());
            return 0;                                                               // gesture not executed successfully
        }

        // Point the right arm to the pointing coordinates
        right_arm_pointing(arm_topic, shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw, duration, interpolation, debug);
        
        // Move the head back to the home position 
        overt_attention_srv.request.state = "disabled";
        overt_attention_srv.request.location_x = 0.0;
        overt_attention_srv.request.location_y = 0.0;
        overt_attention_srv.request.location_z = 0.0;

        // Call the overtAttention/set_mode service to move the head back to the home position
        overt_attention_client.call(overt_attention_srv);

        if(!overt_attention_srv.response.mode_set_success){
            ROS_WARN("%s: deictic gesture executed successfully, but error returning the head to the home position.", node_name.c_str());
        }


        // Move the right arm back to the home position
        send_arm_to_home = go_to_home("RArm", topics_file, interpolation, debug);
        if(send_arm_to_home == 0){
            ROS_WARN("%s: deictic gesture executed successfully, but error returning the right arm to the home position.", node_name.c_str());
        }
    }
    else if(pointing_arm == LEFT_ARM){
        // Extract the topic for the left arm
        if(extract_topic("LArm", topics_file, &arm_topic)){
            return 0;
        }

        // Extract the topic for the left hand
        if(extract_topic("LHand", topics_file, &hand_topic)){
            return 0;
        }

        // Set the joint angles for the left arm
        elbow_yaw = -1.5620;                                                        // Value for elbowYaw for pointing gesture
        elbow_roll = -0.0;                                                          // Value for elbowRoll for pointing gesture
        wrist_yaw = 0.06592;                                                        // Value for wristYaw for pointing gesture

        // Move the arm to the home position
        int send_arm_to_home;
        send_arm_to_home = go_to_home("LArm", topics_file, interpolation, debug);
        if(send_arm_to_home == 0){
            ROS_ERROR("%s: error moving the left arm to the home position.", node_name.c_str());
            return 0;                                                               // gesture not executed successfully
        }

        // Call the overtAttention/set_mode service
        overt_attention_client.call(overt_attention_srv);

        if(!overt_attention_srv.response.mode_set_success){
            ROS_ERROR("%s: error setting the mode for overt attention.", node_name.c_str());
            return 0;                                                               // gesture not executed successfully
        }

        // Point the left arm to the pointing coordinates
        left_arm_pointing(arm_topic, shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw, duration, interpolation, debug);

        // Move the head back to the home position
        overt_attention_srv.request.state = "disabled";
        overt_attention_srv.request.location_x = 0.0;
        overt_attention_srv.request.location_y = 0.0;
        overt_attention_srv.request.location_z = 0.0;

        // Call the overtAttention/set_mode service to move the head back to the home position
        overt_attention_client.call(overt_attention_srv);
        if(!overt_attention_srv.response.mode_set_success){
            ROS_WARN("%s: deictic gesture executed successfully, but error returning the head to the home position.", node_name.c_str());
        }


        // Move the left arm back to the home position
        send_arm_to_home = go_to_home("LArm", topics_file, interpolation, debug);
        if(send_arm_to_home == 0){
            ROS_WARN("%s: deictic gesture executed successfully, but error returning the left arm to the home position.", node_name.c_str());
        }

    }

    // Return the robot to the original orientation if it was rotated
    if(!pose_achievable){
        rotate_robot(-rotation_angle, velocity_publisher, debug);
    }

    return 1;                                                                       // gesture executed successfully
}

/*  
 *   Function to execute iconic gestures. The iconic gestures are executed by moving the robot's arms to the waypoints specified in the gesture descriptors file.
 *   The waypoints are the joint angles for the arms at each point in time.
 *
 *   @param:
 *       gesture_arm_1: the gesture arm 1
 *       waypoints_arm_1: vector to store the waypoints of arm 1
 *       gesture_arm_2: the gesture arm 2
 *       waypoints_arm_2: vector to store the waypoints of arm 2
 *       open_right_hand: boolean to indicate if the right hand should be open
 *       open_left_hand: boolean to indicate if the left hand should be open
 *       gesture_duration: integer to store the gesture duration
 *       topics_file: string to store the topics filename
 *       interpolation: integer to store the interpolation type
 *       debug: boolean to store the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int iconic_gestures(string gesture_arm_1, std::vector<std::vector<double>> waypoints_arm_1, 
                    string gesture_arm_2, std::vector<std::vector<double>> waypoints_arm_2, 
                    bool open_right_hand, bool open_left_hand, int gesture_duration, 
                    string topics_file, int interpolation, bool debug){
    // Declare variables
    int number_of_joints;                                                           // stores the number of joints
    std::vector<std::string> joint_names_arm_1;                                     // vector to store the joint names of arm 1
    std::vector<std::string> joint_names_arm_2;                                     // vector to store the joint names of arm 2
    bool open_hand;                                                                 // boolean to indicate if the hand should be open
    std::string hand = "";                                                          // string to store the hand that should be open or closed

    // Compute the number of joints
    number_of_joints = waypoints_arm_1[0].size();

    // Check if the gesture is for a single arm or both arms
    if (gesture_arm_2 == ""){                                                       // Single arm gesture
        if (gesture_arm_1 == "RArm"){                                               // Right arm gesture
            joint_names_arm_1 = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};
            open_hand = open_right_hand;
            hand = "RHand";
        }
        else if (gesture_arm_1 == "LArm"){                                          // Left arm gesture
            joint_names_arm_1 = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};
            open_hand = open_left_hand;
            hand = "LHand";
        }
        // Move the single arm to the waypoints
        int arm_moved;
        arm_moved = move_single_arm_iconic(gesture_arm_1, joint_names_arm_1, waypoints_arm_1, open_hand, hand, gesture_duration, topics_file, interpolation, debug);
        if(arm_moved == 0){
            ROS_ERROR("%s: error moving the %s to the waypoints.", node_name.c_str(), gesture_arm_1.c_str());
            return 0;                                                               // gesture not executed successfully
        }
    }
    else{                                                                           // Both arms gesture
        if (gesture_arm_1 == "RArm"){                                               // Right arm gesture in arm 1
            joint_names_arm_1 = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};
            joint_names_arm_2 = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};
        }
        else if (gesture_arm_1 == "LArm"){                                          // Left arm gesture in arm 1
            joint_names_arm_1 = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};
            joint_names_arm_2 = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};
        }
        // Move both arms to the waypoints
        
        int arm_moved;
        arm_moved = move_both_arms_iconic(gesture_arm_1, gesture_arm_2, joint_names_arm_1, joint_names_arm_2, waypoints_arm_1, waypoints_arm_2, open_right_hand, open_left_hand, gesture_duration, topics_file, interpolation, debug);   
        if(arm_moved == 0){
            ROS_ERROR("%s: error moving both arms to the waypoints.", node_name.c_str());
            return 0;                                                               // gesture not executed successfully
        }
    }
    return 1;                                                                       // Return 1 if successful
}

/*  
 *   Function to execute bowing gesture
 *   The robot executes a bowing gesture by moving the leg to a specified angle.
 *
 *   @param:
 *       bow_angle: the angle of the leg
 *       gesture_duration: the duration of the gesture
 *       topics_file: the topics filename
 *       interpolation: the interpolation type
 *       debug: the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int bowing_gesture(int bow_angle, int gesture_duration, string topics_file, int interpolation, bool debug){
    // Gesture duration in milliseconds
    double duration = gesture_duration/1000.0;

    // Extract the topic for the leg
    std::string leg_topic;
    if(extract_topic("Leg", topics_file, &leg_topic)){
        return 0;
    }

    // Move the leg to the home position
    int send_leg_to_home;
    send_leg_to_home = go_to_home("Leg", topics_file, interpolation, debug);
    if(send_leg_to_home == 0){
        ROS_ERROR("%s: error moving the leg to the home position.", node_name.c_str());
        return 0; // gesture not executed successfully
    }

    // Execute the bowing gesture
    leg_bowing(leg_topic, bow_angle, gesture_duration, interpolation, debug);

    // Move the leg back to the home position
    send_leg_to_home = go_to_home("Leg", topics_file, interpolation, debug);
    if(send_leg_to_home == 0){
        ROS_WARN("%s: bowing gesture executed successfully, but error moving the leg to the home position.", node_name.c_str());
        return 1;                                                                   // gesture executed successfully but with errors moving the leg to the home position
    }

    return 1;                                                                       // gesture executed successfully
}

/*  
 *   Function to execute nodding gesture
 *   The robot executes a nodding gesture by moving the head to a specified angle.
 *
 *   @param:
 *       nod_angle: the angle of the head
 *       gesture_duration: the duration of the gesture
 *       topics_file: the topics filename
 *       interpolation: the interpolation type
 *       debug: the debug mode
 *
 *   @return:
 *       1 if successful, 0 otherwise
 */
int nodding_gesture(int nod_angle, int gesture_duration, string topics_file, int interpolation, bool debug){   
    // Gesture duration in milliseconds
    double duration = gesture_duration/1000.0;

    // Extract the topic for the head
    std::string head_topic;
    if(extract_topic("Head", topics_file, &head_topic)){
        return 0;
    }

    // Move the head to the home position
    int send_head_to_home;
    send_head_to_home = go_to_home("Head", topics_file, interpolation, debug);
    if(send_head_to_home == 0){
        ROS_ERROR("%s: error moving the head to the home position.", node_name.c_str());
        return 0; // gesture not executed successfully
    }

    // Execute the nodding gesture
    head_nodding(head_topic, nod_angle, gesture_duration, interpolation, debug);
    
    // Move the head back to the home position
    send_head_to_home = go_to_home("Head", topics_file, interpolation, debug);
    if(send_head_to_home == 0){
        ROS_WARN("%s: bowing gesture executed successfully, but error moving the head to the home position.", node_name.c_str());
        return 1;                                                                   // gesture executed successfully but with errors moving the head to the home position
    }

    return 1;                                                                       // gesture executed successfully
}





/*  --------------------------------------------------
            UTILITIY CONTROL FUNCTIONS 
    -------------------------------------------------- 
*/

/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status){
    ROS_INFO("%s: press any key to exit ...", node_name.c_str());
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
void prompt_and_continue(){
    ROS_INFO("%s: press X to quit or press any key to continue...", node_name.c_str());
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        ROS_INFO("%s: exiting ..." , node_name.c_str());
       exit(0);
    }
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

    string right_arm_topic;
    string left_arm_topic;
    string leg_topic;

    if(node_initialized){
        // Send the right arm to the home position
        if(go_to_home("RArm", topics_file_name, interpolation_mode, verbose_mode) == 0){
            ROS_ERROR("%s: unable to move the right arm to home position.", node_name.c_str());
        }

        // Send the left arm to the home position
        if(go_to_home("LArm", topics_file_name, interpolation_mode, verbose_mode) == 0){
            ROS_ERROR("%s: unable to move the left arm to home position.", node_name.c_str());
        }

        // Send the leg to the home position
        if(go_to_home("Leg", topics_file_name, interpolation_mode, verbose_mode) == 0){
            ROS_ERROR("%s: unable to move the leg to home position.", node_name.c_str());
        }
    }

    // Shutdown the node
    shutdown_requested = true;

    ROS_ERROR("%s: terminated.", node_name.c_str());
    ros::shutdown();
}