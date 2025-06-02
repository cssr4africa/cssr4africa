/* overtAttentionImplementation.cpp
*
* Author: Muhammed Danso and Adedayo Akinade, Carnegie Mellon University Africa
* Email: mdanso@andrew.cmu.edu, aakinade@andrew.cmu.edu
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

# include "overtAttention/overtAttentionInterface.h"

// Home positions for the robot head
std::vector<double> head_home_position = {0.0, 0.0};                    // Head pitch and yaw

// Joint states of the robot - updated by subscribing to the /sensor_msgs/joint_states topic
std::vector<double> head_joint_states = {0.0, 0.0};                     // Head pitch and yaw

// Coordinates of the robot in the world (x, y, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};                       // x, y, theta

// Head joint angles for the robot control during scanning or social attention
std::vector<double> attention_head_pitch;
std::vector<double> attention_head_yaw;

// Publisher for the velocity commands
ros::Publisher attention_velocity_publisher;

// Declare the publisher of the /overt_attention/mode topic
ros::Publisher overt_attention_mode_pub;

// Variables for the information about the sound source
double angle_of_sound = 0.0;                                            // Stores the angle of the sound source
double previous_angle_of_sound = 0.0;                                   // Stores the previous angle of sound

// Variables for information about the detected faces and sound
bool face_detected = false;                                             // Stores the status of face detection
bool sound_detected = false;                                            // Stores the status of sound detection
bool mutual_gaze_detected = false;                                      // Stores the status of mutual gaze detection

std::vector<int> face_labels;                                           // Stores the labels of the detected faces
int last_seen_label = -1;                                               // Stores the label of the last seen face
int current_label = -1;                                                 // Stores the label of the current face

// Variables for the attention mode set
int attention_mode = ATTENTION_MODE_DISABLED;                           // Stores the attention mode currently set. Default is DISABLED on initialization
double location_x = 0.0;                                                // Stores the x-coordinate of the location to pay attention to
double location_y = 0.0;                                                // Stores the y-coordinate of the location to pay attention to
double location_z = 0.0;                                                // Stores the z-coordinate of the location to pay attention to
bool location_attended_to = false;                                      // Stores the status if the location request has been attended to once

bool social_attention_done = true;                                      // Flag to indicate if social attention is done

int sound_count = 0;                                                    // Counter for the number of times sound has been detected

bool seek_once = false;                                                 // Variable to check if the seek mode has been called at least once

bool scan_once = false;                                                 // Variable to check if the scan mode has been called at least once

bool disabled_once = false;                                             // Variable to check if the disabled mode has been called at least once

// Variables for the seeking attention mode
std::vector<double> seeking_angles = {-40, 0, 40, 0};                   // Scanning angles for the robot head
size_t seeking_index = -1;                                               // Index for the scanning angles
bool forward_seeking = true;                                            // Flag to indicate the direction of iterating through the scanning angles
bool seeking_completed = false;                                         // Flag to indicate if seeking is completed
int seeking_rotation_count = 0;                                         // Counter for the number of times seeking rotation has been called

cssr_system::Status overt_attention_mode_msg;                           // Message to publish the attention mode

// Variables for the saliency features
cv::Mat faces_map;                                                      // Stores the map of the faces detected         
std::vector<std::tuple<double, double, int>> previous_locations;        // Stores locations that won in the WTA
std::vector<std::tuple<double, double, int>> face_locations;            // Stores the locations of the detected faces saliency
cv::Mat camera_image;                                                   // Hold image from the robot camera for scanning mode

// Variables for the image parameters
double vertical_fov;                                                     // Vertical field of view of the camera
double horizontal_fov;                                                   // Horizontal field of view of the camera
int camera_image_width;                                                  // Width of the image
int camera_image_height;                                                 // Height of the image

// Variables for the random number generator in  the social function
std::mt19937 random_generator;
std::uniform_int_distribution<int> random_distribution;

// Configuration parameters
std::string implementation_platform;
std::string camera_type;
int realignment_threshold;
int x_offset_to_head_yaw;
int y_offset_to_head_pitch;
std::string simulator_topics;
std::string robot_topics;
string topics_filename;
int social_attention_mode;
bool verbose_mode;

std::string node_name;                                                 // Stores the name of the node

bool shutdown_requested = false;                                       // Flag to indicate if a shutdown has been requested
bool node_initialized = false;                                         // Flag to indicate if the node has been initialized





/*  --------------------------------------------------
            CALLBACK FUNCTIONS 
    -------------------------------------------------- 
*/

/*
 *   Callback function for the face detection data received from the /face_detection/data topic
 *   The function receives the face detection data and computes the required head angles to look at the detected faces
 */
void face_detection_data_received(const cssr_system::face_detection_msg_file& data_msg){
    size_t message_length = data_msg.centroids.size();                  // Get the number of faces detected

    if (message_length == 0) {                                          // Check if no face is detected
        return;                                                         // Return if no face is detected
    }

    // Clear the face labels, attention_head_pitch and attention_head_yaw
    face_labels.clear();
    attention_head_pitch.clear();
    attention_head_yaw.clear();

    AngleChange angle_change;                                           // Stores the angle change
    double local_image_head_pitch;                                      // Stores the local image head pitch
    double local_image_head_yaw;                                        // Stores the local image head yaw
    int image_number = 0;

    faces_map = cv::Mat::zeros(camera_image_height, camera_image_width, CV_8UC1);                  // Create a zero matrix with the same size and type as the input image
    
    // Store the status of the mutual gaze detection
    bool gaze_detected = false;
    
    // Loop through the detected faces
    for(size_t i = 0; i < message_length; i++){
        // check for mutual gaze with faces within 2 meters
        if (data_msg.mutualGaze[i] == true && data_msg.centroids[i].z <= 2.0) {
            gaze_detected = true;
        } 

        geometry_msgs::Point face_centroid = data_msg.centroids[i];     // Get the centroid of the faces
        double face_centroid_x = face_centroid.x;
        double face_centroid_y = face_centroid.y;

        // Get the angles from the pixel values
        angle_change = get_angles_from_pixel(face_centroid_x, face_centroid_y, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
        
        // Get the head pitch and yaw angles in radians
        local_image_head_pitch = radians(angle_change.delta_pitch);
        local_image_head_yaw = radians(angle_change.delta_yaw);

        // int new_label = data_msg.face_labels[i];
        int new_label = i+1;

        // if (verbose_mode){
        //     ROS_INFO("%s: Face %d detected at (%.2f, %.2f, %.2f) with head pitch %.2f and head yaw %.2f", node_name.c_str(), new_label, face_centroid.x, face_centroid.y, face_centroid.z, local_image_head_pitch, local_image_head_yaw);
        // }

        // if (face_centroid.z <= 0){                                      // Check if the face centroid is less than or equal to zero
        //     continue;                                                   // Skip the current iteration
        // }

        if (face_centroid.z < 1){                                      // Check if the face centroid is less than or equal to zero
            face_centroid.z = 1;                                      // Set the face centroid to 0.1
        }

        double scale_factor = 1.0 / face_centroid.z;                      // Calculate the scale factor

        // Make the point face_centroid_x, face_centroid_y white
        faces_map.at<float>(face_centroid_y, face_centroid_x) = 1 * scale_factor;

        // ROS_INFO("%s: Face on map %.2f", node_name.c_str(), faces_map.at<float>(face_centroid_y, face_centroid_x));

        // Calibrate the head pitch and yaw angles based on the local image angles
        attention_head_pitch.push_back(local_image_head_pitch);
        attention_head_yaw.push_back(local_image_head_yaw);

        //store the face label
        face_labels.push_back(new_label);
    }
    sound_count++;                                                      // Increment the sound count
    social_attention_done = false;                                      // Set the social attention done status to false
    face_detected = true;                                               // Set the face detected status to true

    if(mutual_gaze_detected == false){                                  // Set the mutual gaze detected status based on the information from the face detection if it has been reset to false by the change mode service call
        mutual_gaze_detected = gaze_detected;                           // Set the mutual gaze detected status to true if any face has a mutual gaze
    }
}

/* 
 *   Callback function for the sound localization data received from the /soundDetection/direction topic
 *   The function receives the sound localization data and stores the angle of the sound source
 */
void sound_localization_data_received(const std_msgs::Float32& data_msg){
    if(isnan(std::abs(data_msg.data))){                                 // Check if the sound localization data is NaN
        angle_of_sound = previous_angle_of_sound;                       // Set the angle of sound to the previous angle of sound
        return;
    }
    previous_angle_of_sound = angle_of_sound;                           // Store the previous angle of sound
    angle_of_sound = data_msg.data;                                     // Store the current angle of sound
    angle_of_sound = radians(angle_of_sound);
    
    social_attention_done = false;                                      // Set the social attention done status to false
    sound_detected = true;                                              // Set the sound detected status to true
}

/* 
 *   Callback function for the camera image received from the /camera/color/image_raw topic
 *   The function receives the camera image and converts it to a cv::Mat image
 */
void front_camera_message_received(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;                                        //  declare a pointer to a CvImage

    //  convert to BGR image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    camera_image = cv_ptr->image;

    // Check if the image is empty
    if (camera_image.empty()) {
        ROS_ERROR("Camera image is empty\n");
    }
}

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
bool set_mode(cssr_system::setMode::Request  &service_request, cssr_system::setMode::Response &service_response){
    // Extract request parameters
    string attention_system_state = service_request.state;
    double point_location_x = service_request.location_x;
    double point_location_y = service_request.location_y;
    double point_location_z = service_request.location_z;

    if (verbose_mode){
        ROS_INFO("%s: request to /overtAttention/set_mode service: \
                \n\t\t\t\t\t\t\tstate\t\t\t: %s, \n\t\t\t\t\t\t\tlocation_x\t\t: %.2f, \
                \n\t\t\t\t\t\t\tlocation_y\t\t: %.2f, \n\t\t\t\t\t\t\tlocation_z\t\t: %.2f.",\
                node_name.c_str(), service_request.state.c_str(), service_request.location_x, service_request.location_y, service_request.location_z);
    }

    // Set the attention mode
    if(attention_system_state == ATTENTION_DISABLED_STATE){
        attention_mode = ATTENTION_MODE_DISABLED;
        disabled_once = false;                                          // Set the disabled mode called status
        service_response.mode_set_success = 1;                          // Attention mode set to disabled successfully
    }
    else if(attention_system_state == ATTENTION_SOCIAL_STATE){
        attention_mode = ATTENTION_MODE_SOCIAL;
        service_response.mode_set_success = 1;                          // Attention mode set to social successfully
    } 
    else if(attention_system_state == ATTENTION_SCANNING_STATE){
        attention_mode = ATTENTION_MODE_SCANNING;
        scan_once = false;                                              // Reset the scan mode called status
        service_response.mode_set_success = 1;                          // Attention mode set to scanning successfully
    } 
    else if(attention_system_state == ATTENTION_SEEKING_STATE){
        attention_mode = ATTENTION_MODE_SEEKING;
        seek_once = false;                                              // Reset the seek mode called status
        mutual_gaze_detected = false;                                   // Reset the mutual gaze detected status
        seeking_index = -1;                                              // Reset the seeking index to 0
        seeking_rotation_count = 0;
        seeking_completed = false;
        service_response.mode_set_success = 1;                          // Attention mode set to scanning successfully
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
        ROS_ERROR("%s: Invalid attention system state; supported states are 'disabled', 'seeking', 'social', 'scanning', and 'location'", node_name.c_str());
        attention_mode = ATTENTION_MODE_DISABLED;
        service_response.mode_set_success = 0;                          // Attention mode set unsuccessful
    }

    if (service_response.mode_set_success == 1){                        // Print the response to the service
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
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;   // used to turn debug message on

    std::string data_file = "robotPose.dat";                            // data filename
    std::string data_path;                                              // data path
    std::string data_path_and_file;                                     // data path and filename

    std::string x_key = "x";                                            // x key
    std::string y_key = "y";                                            // y key
    std::string theta_key = "theta";                                    // theta key

    std::string x_value;                                                // x value
    std::string y_value;                                                // y value
    std::string z_value;                                                // z value
    std::string theta_value;                                            // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/overtAttention/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string data_line_read;                                         // variable to read the line in the file
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
        printf("\tX: %f\n", robot_pose_input[0]);
        printf("\tY: %f\n", robot_pose_input[1]);
        printf("\tTheta: %f\n", robot_pose_input[2]);
    }
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
    topic_path += "/overtAttention/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("%s: Unable to open the topic file %s\n", node_name.c_str(), topic_path_and_file.c_str());
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
void set_image_parameters(string platform, string camera, double* vertical_fov, double* horizontal_fov, int* image_width, int* image_height){
    if (camera == "FrontCamera"){                                       // Set the image parameters for the front camera
        *vertical_fov = VFOV_PEPPER_FRONT_CAMERA;                       // Set the vertical field of view
        *horizontal_fov = HFOV_PEPPER_FRONT_CAMERA;                     // Set the horizontal field of view
        *image_width = IMG_WIDTH_PEPPER_FRONT_CAMERA;                   // Set the image width
        *image_height = IMG_HEIGHT_PEPPER_FRONT_CAMERA;                 // Set the image height
    }
    else if (camera == "StereoCamera"){                                 // Set the image parameters for the stereo camera
        *vertical_fov = VFOV_PEPPER_STEREO_CAMERA;                      // Set the vertical field of view
        *horizontal_fov = HFOV_PEPPER_STEREO_CAMERA;                    // Set the horizontal field of view
        *image_width = IMG_WIDTH_PEPPER_STEREO_CAMERA;                  // Set the image width
        *image_height = IMG_HEIGHT_PEPPER_STEREO_CAMERA;                // Set the image height
    }
    else if (camera == "RealSenseCamera"){                              // Set the image parameters for the RealSense camera
        *vertical_fov = VFOV_REALSENSE_CAMERA;                          // Set the vertical field of view
        *horizontal_fov = HFOV_REALSENSE_CAMERA;                        // Set the horizontal field of view
        *image_width = IMG_WIDTH_REALSENSE_CAMERA;                      // Set the image width
        *image_height = IMG_HEIGHT_REALSENSE_CAMERA;                    // Set the image height
    }
    else{
        *vertical_fov = VFOV_PEPPER_FRONT_CAMERA;                       // Set the vertical field of view
        *horizontal_fov = HFOV_PEPPER_FRONT_CAMERA;                     // Set the horizontal field of view
        *image_width = IMG_WIDTH_PEPPER_FRONT_CAMERA;                   // Set the image width
        *image_height = IMG_HEIGHT_PEPPER_FRONT_CAMERA;                 // Set the image height
    }
}


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
int read_configuration_file(string* platform, string* camera, int* realignment_threshold, int* x_offset_to_head_yaw, int* y_offset_to_head_pitch, string* simulator_topics, string* robot_topics, string* topics_filename, int* social_attention_mode, bool* debug_mode){
    std::string config_file = "overtAttentionConfiguration.ini";        // data filename
    std::string config_path;                                            // data path
    std::string config_path_and_file;                                   // data path and filename
     
    std::string platform_key = "platform";                              // platform key 
    std::string camera_key = "camera";                                  // camera key
    std::string realignment_threshold_key = "realignmentThreshold";     // realignment threshold key
    std::string x_offset_to_head_yaw_key = "xOffsetToHeadYaw";          // x offset to head yaw key
    std::string y_offset_to_head_pitch_key = "yOffsetToHeadPitch";      // y offset to head pitch key
    std::string simulator_topics_key = "simulatorTopics";               // simulator topics key
    std::string robot_topics_key = "robotTopics";                       // robot topics key
    std::string social_attention_mode_key = "socialAttentionMode";      // social attention mode key
    std::string verbose_mode_key = "verboseMode";                       // verbose mode key

    std::string platform_value;                                         // platform value 
    std::string camera_value;                                           // camera value
    std::string realignment_threshold_value;                            // realignment threshold value
    std::string x_offset_to_head_yaw_value;                             // x offset to head yaw value
    std::string y_offset_to_head_pitch_value;                           // y offset to head pitch value
    std::string simulator_topics_value;                                 // simulator topics value
    std::string robot_topics_value;                                     // robot topics value
    std::string social_attention_mode_value;                            // social attention mode value
    std::string verbose_mode_value;                                     // verbose mode value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += "/overtAttention/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;

    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("%s: unable to open the configuration file %s\n", node_name.c_str(), config_path_and_file.c_str());
        return 1;
    }

    // Set platform to the default value of robot
    *platform = "robot";
    platform_value = "robot";

    std::string data_line_read;                                         // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key, param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        
        // Extract the platform -- Removed the platform key from the configuration file but will be useful if you need it
        if (param_key == platform_key){ 
            boost::algorithm::to_lower(param_value);                    // modifies string to lower case
            platform_value = param_value;
            *platform = param_value;
            if(platform_value != "robot" && platform_value != "simulator"){
                ROS_WARN("%s: platform value not supported in configuration. Supported values are: robot and simulator.", node_name.c_str());
                return 1;
            }
        }
        
        else if (param_key == camera_key){ 
            camera_value = param_value;
            *camera = param_value;
            if(camera_value != "FrontCamera" && camera_value != "StereoCamera" && camera_value != "RealSenseCamera"){
                ROS_WARN("%s: camera value not supported in configuration. Supported values are: FrontCamera, RealSenseCamera and StereoCamera.", node_name.c_str());
                return 1;
            }
        }

        else if (param_key == realignment_threshold_key){ 
            realignment_threshold_value = param_value;
            *realignment_threshold = std::stoi(param_value);
        }

        else if (param_key == x_offset_to_head_yaw_key){ 
            x_offset_to_head_yaw_value = param_value;
            *x_offset_to_head_yaw = std::stoi(param_value);
        }

        else if (param_key == y_offset_to_head_pitch_key){ 
            y_offset_to_head_pitch_value = param_value;
            *y_offset_to_head_pitch = std::stoi(param_value);
        }

        else if (param_key == simulator_topics_key){ 
            simulator_topics_value = param_value;
            *simulator_topics = param_value;
        }

        else if (param_key == robot_topics_key){ 
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }
        
        else if(param_key == social_attention_mode_key){
            social_attention_mode_value = param_value;
            if(social_attention_mode_value == "saliency"){
                *social_attention_mode = SALIENCY_SOCIAL_CONTROL;
            }
            else if(social_attention_mode_value == "random"){
                *social_attention_mode = RANDOM_SOCIAL_CONTROL;
            }
            else{
                ROS_WARN("%s: social attention mode value not supported. Supported values are: saliency and random", node_name.c_str());
                return 1;
            }
        }

        else if (param_key == verbose_mode_key){ 
            boost::algorithm::to_lower(param_value);                    // modifies string to lower case
            verbose_mode_value = param_value;
            if(verbose_mode_value == "true"){
                *debug_mode = true;
            }
            else if(verbose_mode_value == "false"){
                *debug_mode = false;
            }
            else{
                ROS_WARN("%s: verbose mode value not supported in configuration. Supported values are: true and false.", node_name.c_str());
                return 1;
            }
        }
    }
    data_if.close();

    if(*platform == "" || *camera == "" || *simulator_topics == "" || *robot_topics == ""){
        ROS_WARN("%s: unable to find a valid configuration. Please check the configuration file:\n\t\t\t\t\t\t '%s'.", node_name.c_str(), config_path_and_file.c_str());
        return 1;
    }

    if (platform_value == "robot"){
        *topics_filename = *robot_topics;
    }
    else if(platform_value == "simulator"){
        *topics_filename = *simulator_topics;
    }

    print_configuration(*platform, *camera, *realignment_threshold, *x_offset_to_head_yaw, *y_offset_to_head_pitch, *simulator_topics, *robot_topics, *topics_filename, *debug_mode);
    return 0;
}

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
void print_configuration(string platform, string camera, int realignment_threshold, int x_offset_to_head_yaw, int y_offset_to_head_pitch, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode){
    // Print the gesture execution configuration
    ROS_INFO("%s: configuration parameters: \
                \n\t\t\t\t\t\t\tImplementation platform\t\t: %s, \n\t\t\t\t\t\t\tCamera\t\t\t\t: %s, \
                \n\t\t\t\t\t\t\tRealignment Threshold\t\t: %d, \n\t\t\t\t\t\t\tX Offset to Head Yaw\t\t: %d, \
                \n\t\t\t\t\t\t\tY Offset to Head Pitch\t\t: %d, \n\t\t\t\t\t\t\tSimulator Topics file\t\t: %s, \
                \n\t\t\t\t\t\t\tRobot Topics file\t\t: %s, \n\t\t\t\t\t\t\tVerbose Mode\t\t\t: %s.",\
                node_name.c_str(), implementation_platform.c_str(), camera.c_str(), realignment_threshold, \
                x_offset_to_head_yaw, y_offset_to_head_pitch, simulator_topics.c_str(), robot_topics.c_str(), \
                debug_mode ? "true" : "false");
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

AngleChange get_angles_from_pixel(double center_x, double center_y, double image_width, double image_height, double theta_h, double theta_v){
    // Calculate the offsets from the image center
    double x_offset = center_x - image_width / 2.0;
    double y_offset = center_y - image_height / 2.0;
    
    // Calculate the proportion of the offset relative to the image dimensions
    double x_proportion = x_offset / image_width;
    double y_proportion = y_offset / image_height;
    
    // Calculate the angle changes
    double delta_yaw = x_proportion * theta_h * -1;
    double delta_pitch = y_proportion * theta_v;
    
    // Return the angle changes
    return {delta_yaw, delta_pitch};
}

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
PixelCoordinates calculate_pixel_coordinates(double delta_yaw, double delta_pitch, int W, int H, double theta_h, double theta_v) {
    // Calculate the proportions of the angles within the field of view
    double x_proportion = delta_yaw / radians(theta_h) * -1;
    double y_proportion = delta_pitch / radians(theta_v);
    
    // Calculate the pixel offsets from the center
    double x_offset = x_proportion * W;
    double y_offset = y_proportion * H;
    
    // Calculate the pixel coordinates by adding the offsets to the image center
    int x = static_cast<int>(W / 2.0 + x_offset);
    int y = static_cast<int>(H / 2.0 + y_offset);
    
    // Return the pixel coordinates
    return {x, y};
}





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
std::pair<int, int> winner_takes_all(const cv::Mat& saliency_map) {
    // Declare the variables to store the minimum and maximum values and their locations
    double min_val;
    double max_val;
    cv::Point min_loc;
    cv::Point max_loc;

    // Find the minimum and maximum values and their locations
    cv::minMaxLoc(saliency_map, &min_val, &max_val, &min_loc, &max_loc);

    // Return the coordinates of the maximum point (col, row)
    return {max_loc.x, max_loc.y};
}

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
std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> habituation(cv::Mat& saliency_map, cv::Mat& wta_map, const std::vector<std::tuple<double, double, int>>& previous_locations) {
    std::vector<std::tuple<double, double, int>> prev;
    
    for (const auto& loc : previous_locations) {
        double x_robot = std::get<0>(loc) - head_joint_states[1];
        double y_robot = std::get<1>(loc) - head_joint_states[0];

        // Calculate pixel coordinates based on robot head position and field of view
        PixelCoordinates pixel_coordinates = calculate_pixel_coordinates(x_robot, y_robot, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
        int x = pixel_coordinates.x;
        int y = pixel_coordinates.y;

        // Check if the coordinates are within bounds
        if (x >= 0 && x < camera_image_width && y >= 0 && y < camera_image_height) {
            // Gradually reduce the saliency in the previously attended locations
            for (int dx = -PATCH_RADIUS; dx <= PATCH_RADIUS; dx++) {
                for (int dy = -PATCH_RADIUS; dy <= PATCH_RADIUS; dy++) {
                    int x_patch = x + dx;
                    int y_patch = y + dy;

                    // Ensure the patch is within bounds
                    if (x_patch >= 0 && x_patch < camera_image_width && y_patch >= 0 && y_patch < camera_image_height) {
                        saliency_map.at<float>(y_patch, x_patch) -= std::get<2>(loc) * HABITUATION_RATE;
                    }
                }
            }

            // Draw a circle around the patch in the wtaMap
            cv::circle(wta_map, cv::Point(x, y), PATCH_RADIUS, std::get<2>(loc) * PATCH_RADIUS, -1);
        }

        // Update the previous locations list by incrementing the third element (time or iterations)
        prev.push_back(std::make_tuple(std::get<0>(loc), std::get<1>(loc), std::get<2>(loc) + 1));
    }

    // Return the updated saliency map and previous locations
    return {saliency_map, prev};
}

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
std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> inhibition_of_return(cv::Mat& saliency_map, cv::Mat& wta_map, const std::vector<std::tuple<double, double, int>>& previous_locations) {
    std::vector<std::tuple<double, double, int>> prev;

    for (const auto& loc : previous_locations) {
        double time = std::get<2>(loc);  // Time or iterations for each location

        // Check if the location has exceeded the habituation timeout
        if (time > IOR_LIMIT && time < IOR_LIMIT+50) {
            double x_robot = std::get<0>(loc) - head_joint_states[1];
            double y_robot = std::get<1>(loc) - head_joint_states[0];

            // Calculate pixel coordinates based on robot head position and field of view
            PixelCoordinates pixel_coordinates = calculate_pixel_coordinates(x_robot, y_robot, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
            int x = pixel_coordinates.x;
            int y = pixel_coordinates.y;

            // Ensure the coordinates are within bounds
            if (x >= 0 && x < camera_image_width && y >= 0 && y < camera_image_height) {
                // Set the previously attended region to zero in the saliency map
                for (int dx = -PATCH_RADIUS; dx <= PATCH_RADIUS; dx++) {
                    for (int dy = -PATCH_RADIUS; dy <= PATCH_RADIUS; dy++) {
                        int x_patch = x + dx;
                        int y_patch = y + dy;

                        // Ensure the patch is within bounds
                        if (x_patch >= 0 && x_patch < camera_image_width && y_patch >= 0 && y_patch < camera_image_height) {
                            saliency_map.at<float>(y_patch, x_patch) = 0;
                        }
                    }
                }

                // Suppress the region in the WTA map
                cv::circle(wta_map, cv::Point(x, y), PATCH_RADIUS, 0, -1);
            }
        } else {
            // Keep locations that have not exceeded the habituation timeout
            prev.push_back(loc);
        }
    }

    // Return the updated saliency map and previous locations
    return {saliency_map, prev};
}

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
int compute_saliency_features(cv::Mat camera_image, int* centre_x, int* centre_y, bool debug_mode){
    cv::Mat wta = cv::Mat::zeros(camera_image.size(), camera_image.type());  // Create a zero matrix with the same size and type as the input image

    // Set all channels (B, G, R) to 255 (white)
    wta.forEach<cv::Vec3b>([](cv::Vec3b &pixel, const int * position) -> void {
        pixel[0] = 255; // Blue channel
        pixel[1] = 255; // Green channel
        pixel[2] = 255; // Red channel
    });

    // Create the StaticSaliencyFineGrained object
    cv::Ptr<cv::saliency::StaticSaliencyFineGrained> saliency = cv::saliency::StaticSaliencyFineGrained::create();

    // Compute the saliency map
    cv::Mat saliency_map;
    bool success = saliency->computeSaliency(camera_image, saliency_map);

    if (!success) {                                                         // Check if the saliency map was computed successfully
        ROS_ERROR("%s: Error computing saliency map.", node_name.c_str());  // Print an error message
        return -1;                                                          // Return -1 to indicate an error
    }

    // Project detect faces onto saliency map
    for(size_t i = 0; i < face_labels.size(); i++){
        double pitch = attention_head_pitch[i];
        double yaw = attention_head_yaw[i];

        PixelCoordinates pixel_coordinates = calculate_pixel_coordinates(yaw, pitch, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
        int x = pixel_coordinates.x;
        int y = pixel_coordinates.y;

        // Check if the coordinates are within bounds
        if (x >= 0 && x < camera_image_width && y >= 0 && y < camera_image_height) {
            saliency_map.at<float>(y, x) = 2;
        }
    }

    // Clear the face labels, attention_head_pitch and attention_head_yaw
    face_labels.clear();
    attention_head_pitch.clear();
    attention_head_yaw.clear();

    // Display the saliency map
    if(debug_mode){
        cv::imshow("Saliency Map", saliency_map);
        cv::waitKey(1);
    }

    // Apply Habituation (reduce response of previously attended points)
    std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> hab_out = habituation(saliency_map, wta, previous_locations);
    // saliencyMap = hab_out.first;
    previous_locations = hab_out.second;
    
    // Apply Inhibition of Return (suppress previously attended regions)
    std::pair<cv::Mat, std::vector<std::tuple<double, double, int>>> ior_out = inhibition_of_return(saliency_map, wta, previous_locations);
    // saliencyMap = ior_out.first;
    previous_locations = ior_out.second;

    // Obtain the coordinates of the maximum saliency point
    std::pair<int, int> maxPoint = winner_takes_all(saliency_map);
    *centre_x = maxPoint.first;
    *centre_y = maxPoint.second;
    
    // Draw the saliency point on the WTA map
    if(debug_mode){
        // Draw the first diagonal line (green, thickness of 5)
        cv::line(wta, cv::Point(*centre_x - 15, *centre_y - 15), cv::Point(*centre_x + 15, *centre_y + 15), cv::Scalar(0, 255, 0), 5);

        // Draw the second diagonal line (green, thickness of 5)
        cv::line(wta, cv::Point(*centre_x - 15, *centre_y + 15), cv::Point(*centre_x + 15, *centre_y - 15), cv::Scalar(0, 255, 0), 5);

        // Display the original image
        cv::imshow("Image", wta);

        cv::waitKey(1);
    }

    // Return 0 to indicate success
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
    // Compute the trajectory for the last point in time
    // time_t = trajectory_duration;
    // for(int i = 0; i < number_of_joints; i++){                          // Create a trajectory for each joint (5 joints for the arm)   
    //     position = start_position[i] + (end_position[i] - start_position[i]) * ((10 * (pow(time_t/trajectory_duration, 3))) - (15 * (pow(time_t/trajectory_duration, 4))) + (6 * (pow(time_t/trajectory_duration, 5))));
    //     positions_t.push_back(position);

    //     velocity = ((end_position[i] - start_position[i])/trajectory_duration) * ((30 * (pow(time_t/trajectory_duration, 2))) - (60 * (pow(time_t/trajectory_duration, 3))) + (30 * (pow(time_t/trajectory_duration, 4))));
    //     velocities_t.push_back(velocity);

    //     acceleration = ((end_position[i] - start_position[i])/(trajectory_duration*trajectory_duration)) * ((60 * (pow(time_t/trajectory_duration, 1))) - (180 * (pow(time_t/trajectory_duration, 2))) + (120 * (pow(time_t/trajectory_duration, 3))));
    //     accelerations_t.push_back(acceleration);
    // }
    // // Store the computed trajectory for the last point in time
    // positions.push_back(positions_t);
    // velocities.push_back(velocities_t);
    // accelerations.push_back(accelerations_t);
    // durations.push_back(time_t);

    return;
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
        ROS_ERROR("%s: error creating action client for head controller", node_name.c_str());
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
        ROS_ERROR("%s: error creating action client for head controller", node_name.c_str());
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
                        std::vector<double> positions, bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Configure parameters for rotating the robot
    geometry_msgs::Twist velocity_command;                              // Declare a geometry_msgs::Twist message to send velocity commands to the robot

    ros::Rate loop_rate(10);                                            // Set publishing rate to 10 Hz        

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Send the goal to move the head to the specified position
    head_client->sendGoal(goal);

    // Rotate the robot by the specified angle
    if(rotate_robot){
        // Publish the velocity command to the robot
        velocity_publisher.publish(velocity_command);

        // // Sleep for the duration of the rotation
        // loop_rate.sleep();
    }
    head_client->waitForResult(ros::Duration(duration));                // Wait for the actuator to reach the specified position
}

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
                                        bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
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

    // Configure parameters for rotating the robot
    geometry_msgs::Twist velocity_command;                              // Declare a geometry_msgs::Twist message to send velocity commands to the robot

    ros::Rate loop_rate(5);                                             // Set publishing rate to 10 Hz        

    // Set the linear velocities to zero and angular velocity to the angle in radian
    velocity_command.linear.x = 0.0;
    velocity_command.linear.y = 0.0;
    velocity_command.linear.z = 0.0;

    velocity_command.angular.x = 0.0;
    velocity_command.angular.y = 0.0;
    velocity_command.angular.z = angle_radians;

    // Send the goal to move the head to the specified position
    client->sendGoal(goal);

    // Rotate the robot by the specified angle
    if(rotate_robot){
        // Publish the velocity command to the robot
        velocity_publisher.publish(velocity_command);

        // Sleep for the duration of the rotation
        // loop_rate.sleep();
    }

    client->waitForResult(ros::Duration(gesture_duration));             // Wait for the actuator to reach the specified position
}

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
                            bool rotate_robot, double angle_radians, ros::Publisher velocity_publisher, bool debug){
    // Create a control client for the head
    ControlClientPtr head_client = create_client(head_topic);
    if(head_client == nullptr){
        ROS_ERROR("%s: error creating action client for head controller", node_name.c_str());
        return;
    }

    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};// Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
    std::vector<double> head_position = {head_pitch, head_yaw};
    std::vector<double> head_current_position = head_joint_states;

    // Vectors to store the positions, velocities, accelerations and duration of the trajectory
    std::vector<std::vector<double>> positions_t_head;
    std::vector<std::vector<double>> velocities_t_head;
    std::vector<std::vector<double>> accelerations_t_head;
    std::vector<double> duration_t_head;

    // Compute the trajectory for the head movement
    // compute_trajectory(head_joint_states, head_position, number_of_joints, gesture_duration, positions_t_head, velocities_t_head, accelerations_t_head, duration_t_head);

    // // Move the head and wheels to the specified position
    // move_robot_head_wheels_to_position_biological_motion(head_client, head_joint_names, gesture_duration, duration_t_head, positions_t_head, velocities_t_head, accelerations_t_head, rotate_robot, angle_radians, velocity_publisher, debug);

    // Move the head and wheels to the specified position
    move_robot_head_wheels_to_position(head_client, head_joint_names, gesture_duration, head_position, rotate_robot, angle_radians, velocity_publisher, debug);

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
    /* */

    // Calculate the camera coordinates
    double distance = sqrt(pow((location_x - head_x), 2) + pow((location_y - head_y), 2) + pow((location_z - head_z), 2));
    l_head_2 = distance - l_head_1;

    camera_x = ((l_head_1 * location_x) + (l_head_2 * head_x))/(l_head_1 + l_head_2);
    camera_y = ((l_head_1 * location_y) + (l_head_2 * head_y))/(l_head_1 + l_head_2);
    camera_z = ((l_head_1 * location_z) + (l_head_2 * head_z))/(l_head_1 + l_head_2);
    camera_z = camera_z + 61.6 - TORSO_HEIGHT;

    // Variables to store the head angles
    double head_yaw; 
    double head_pitch;

    get_head_angles(camera_x, camera_y, camera_z, &head_yaw, &head_pitch);
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
int social_attention(std::string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, int social_control, bool debug){
    std::string head_topic;                                             // stores the head topic
    // Extract the topic for the head
    if(extract_topic("Head", topics_file, &head_topic) != 0){
        return -1;                                                      // return -1 if the head topic is not extracted successfully
    }

    double control_head_pitch;
    double control_head_yaw;
    double current_attention_head_pitch = 0.0;
    double current_attention_head_yaw = 0.0;
    double realignment_threshold_radians = radians(realignment_threshold);
    bool realign_robot_base = false;
    double robot_rotation_angle = 0.0;

    if(face_detected || sound_detected){
        if(face_detected){
            if(social_control == RANDOM_SOCIAL_CONTROL){                // Random social control
                if (face_labels.size() == 1){
                    current_attention_head_pitch = attention_head_pitch[0];
                    current_attention_head_yaw = attention_head_yaw[0];
                    last_seen_label = -1;
                }
                else{
                    random_distribution = std::uniform_int_distribution<int>(0, face_labels.size()-1);
                    int random_index;
                    int chosen_label;
                    do{
                        random_index = random_distribution(random_generator);
                        chosen_label = face_labels[random_index];
                    }
                    while((chosen_label == last_seen_label));

                    last_seen_label = chosen_label;

                    current_attention_head_pitch = attention_head_pitch[random_index];
                    current_attention_head_yaw = attention_head_yaw[random_index];
                }
            }

            else{                                                       // Saliency social control
                if (sound_detected){
                    // Calculate pixel coordinates based on sound localization angle
                    PixelCoordinates pixel_coordinates = calculate_pixel_coordinates(angle_of_sound, 0.0, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
                    int sound_x = pixel_coordinates.x;
                    int sound_y = pixel_coordinates.y;

                    if (sound_x >= 0 && sound_x < camera_image_width && sound_y >= 0 && sound_y < camera_image_height){
                        faces_map.at<float>(sound_y, sound_x) = 2;
                    }
                }
                
                // Habituating previously attended faces
                for (const auto& loc : face_locations) {
                    double x_robot = std::get<0>(loc) - head_joint_states[1];
                    double y_robot = std::get<1>(loc) - head_joint_states[0];

                    // Calculate pixel coordinates based on robot head position and field of view
                    PixelCoordinates pixel_coordinates = calculate_pixel_coordinates(x_robot, y_robot, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
                    int x = pixel_coordinates.x;
                    int y = pixel_coordinates.y;

                    // Check if the coordinates are within bounds
                    if (x >= 0 && x < camera_image_width && y >= 0 && y < camera_image_height) {
                        // Gradually reduce the saliency in the previously attended locations
                        // cv::circle(faces_map, cv::Point(x, y), 1, 250 - std::get<2>(loc) * 100, -1);
                        // Make the point face_centroid_x, face_centroid_y white
                        faces_map.at<float>(y, x) = 1 - std::get<2>(loc) * 0.5;
                    }
                }

                // IOR
                for (size_t i = 0; i < face_locations.size(); i++) {
                    if (std::get<2>(face_locations[i]) < 2) {
                        face_locations.push_back(std::make_tuple(std::get<0>(face_locations[i]), std::get<1>(face_locations[i]), std::get<2>(face_locations[i])+1));
                    }
                    // Remove the element at index i
                    face_locations.erase(face_locations.begin() + i);
                }

                // Display the faces map
                if(debug){
                    cv::imshow("Faces Map", faces_map);
                    cv::waitKey(1);
                }

                // Get the maximum point in the faces map
                std::pair<int, int> maxPoint = winner_takes_all(faces_map);
                int x = maxPoint.first;
                int y = maxPoint.second;
                // ROS_INFO("%s: Max Point: (X: %d, Y: %d)", node_name.c_str(), x, y);

                // Calculate the angles from the pixel coordinates
                AngleChange angle_change = get_angles_from_pixel(x, y, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
                double winner_head_pitch = radians(angle_change.delta_pitch);
                double winner_head_yaw = radians(angle_change.delta_yaw);

                // Update the face locations
                face_locations.push_back(std::make_tuple(winner_head_yaw + head_joint_states[1], winner_head_pitch + head_joint_states[0], 1));
                
                // set attention to the winning point
                current_attention_head_pitch = winner_head_pitch;
                current_attention_head_yaw = winner_head_yaw;
            }
            current_attention_head_pitch -= radians(y_offset_to_head_pitch);
            current_attention_head_yaw -= radians(x_offset_to_head_yaw);
        }

        // Set the control head pitch and yaw
        control_head_pitch = head_joint_states[0];
        control_head_yaw = head_joint_states[1];
        // ROS_INFO("%s: Joint States: Head Pitch: %f, Head Yaw: %f", node_name.c_str(), control_head_pitch, control_head_yaw);


        if(face_detected && sound_detected){                            // When a face is detected and sound is detected
            if(social_control == RANDOM_SOCIAL_CONTROL) {
                if (sound_count > 20)                                       // If the sound count is greater than 20 set the head pitch to 0 and head yaw to the angle of sound
                {
                    control_head_pitch = DEFAULT_HEAD_PITCH;
                    control_head_yaw += angle_of_sound;
                    sound_count = 0;                                        // Reset the sound count        
                } else {                                                    // If the sound count is less than 20 set the head pitch and head yaw to the current attention head pitch and yaw
                    control_head_pitch += current_attention_head_pitch;
                    control_head_yaw += current_attention_head_yaw;
                }
            } else {
                control_head_pitch += current_attention_head_pitch;
                control_head_yaw += current_attention_head_yaw;
            }
        }

        else if(face_detected && !sound_detected){                      // If a face is detected and sound is not detected
            control_head_pitch += current_attention_head_pitch;
            control_head_yaw += current_attention_head_yaw;
        }

        else if(!face_detected && sound_detected){                      // If a face is not detected and sound is detected
            control_head_pitch = DEFAULT_HEAD_PITCH;
            control_head_yaw += angle_of_sound;
            sound_count = 0;                                            // Reset the sound count            
        }
        else{
            return 0;
        }

        // Check if the head pitch is within the specified bounds
        if(control_head_pitch >= MAX_HEAD_PITCH){
            control_head_pitch = MAX_HEAD_PITCH;
        }
        else if(control_head_pitch <= MIN_HEAD_PITCH){
            control_head_pitch = MIN_HEAD_PITCH;
        }
        
        // ROS_INFO("%s: Control Head Pitch: %f, Control Head Yaw: %f", node_name.c_str(), control_head_pitch, control_head_yaw);
        // Check if the head yaw is within the specified bounds and realign the robot base
        if((control_head_yaw >= realignment_threshold_radians) || (control_head_yaw <= -realignment_threshold_radians)){
            robot_rotation_angle = control_head_yaw;
            control_head_yaw = 0;
            realign_robot_base = true;
        }

        // Ensure the robot looks up slightly  -- Comment out if not needed
        if(control_head_pitch >= DEFAULT_HEAD_PITCH){
            control_head_pitch = DEFAULT_HEAD_PITCH;
        }

        // Move the robot's head and wheels to the specified position
        control_robot_head_wheels(head_topic, control_head_pitch, control_head_yaw, 0.5, realign_robot_base, robot_rotation_angle, velocity_publisher, debug);
        ros::Duration(1).sleep();
        
        // Reset the face and sound detection flags
        face_detected = false;
        sound_detected = false;

    }

    // Clear the face labels, attention_head_pitch and attention_head_yaw
    face_labels.clear();
    attention_head_pitch.clear();
    attention_head_yaw.clear();

    return 1;                                                           // attention executed successfully  
}

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
int scanning_attention(double control_head_yaw, double control_head_pitch, string topics_file, ros::Publisher velocity_publisher, bool debug){

    string head_topic;
    // Extract the topic for the head
    if(extract_topic("Head", topics_file, &head_topic) != 0){
        return -1;                                                      // return -1 if the head topic is not extracted successfully
    }
    
    control_head_pitch -= radians(y_offset_to_head_pitch);
    control_head_yaw -= radians(x_offset_to_head_yaw);

    // Check if the head yaw and head pitch are within the specified bounds
    if(control_head_yaw >= MAX_HEAD_YAW_SCANNING){
        control_head_yaw = MAX_HEAD_YAW_SCANNING;
    }
    else if(control_head_yaw <= MIN_HEAD_YAW_SCANNING){
        control_head_yaw = MIN_HEAD_YAW_SCANNING;
    }

    if(control_head_pitch >= MAX_HEAD_PITCH_SCANNING){
        control_head_pitch = MAX_HEAD_PITCH_SCANNING;
    }
    else if(control_head_pitch <= MIN_HEAD_PITCH_SCANNING){
        control_head_pitch = MIN_HEAD_PITCH_SCANNING;
    }

    // Move the robot's head to scan the environment based on the specified head pitch and head yaw
    // if(move_robot_head_biological_motion(head_topic, control_head_pitch, control_head_yaw, 1.0, debug) != 0){
    //     ROS_WARN("%s: error moving the robot's head to the scanning location", node_name.c_str());
    //     return -1;                                                      // return -1 if the robot's head is not moved to the specified scanning location
    // }
    if(move_robot_head(head_topic, control_head_pitch, control_head_yaw, 0.5, debug) != 0){
        ROS_WARN("%s: error moving the robot's head to the scanning location", node_name.c_str());
        return -1;                                                      // return -1 if the robot's head is not moved to the specified scanning location
    }
    ros::Duration(1).sleep();

    // Clear the face labels, attention_head_pitch and attention_head_yaw
    face_labels.clear();
    attention_head_pitch.clear();
    attention_head_yaw.clear();
    
    return 1;
}

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
int seeking_attention(string topics_file, int realignment_threshold, ros::Publisher velocity_publisher, ros::Publisher overt_attention_mode_pub, bool debug){
    double next_angle;
    double control_head_yaw;
    double control_head_pitch = DEFAULT_HEAD_PITCH;

    string head_topic;
    // Extract the topic for the head
    if(extract_topic("Head", topics_file, &head_topic) != 0){
        return -1;                                                      // return -1 if the head topic is not extracted successfully
    }

    double seek_angle;                                                  // Get the current angle to seek
    double rotation_angle = 90.0;

    if ((seeking_index != -1) && (seeking_index >= seeking_angles.size())) {
        seeking_index = 0;
        rotate_robot(rotation_angle, velocity_publisher, debug);
        seeking_rotation_count++;
    }

    if(seeking_index == -1){                                            // If the seeking index is -1 when the seek mode is activated
        control_head_yaw = head_joint_states[1];                        // Set the head yaw to the current head yaw measured from joint states
    }
    else{
        seek_angle = seeking_angles[seeking_index % 4];                 // Get the current angle to seek
        control_head_yaw = radians(seek_angle);                         // Convert the angle to radians
    }

    seeking_index++;

    if(seeking_rotation_count >= 4){
        seeking_completed = true;
        seeking_rotation_count = 0;
        return 1;
    }
        

    if(control_head_yaw >= MAX_HEAD_YAW){
        control_head_yaw = MAX_HEAD_YAW;
    }
    else if(control_head_yaw <= MIN_HEAD_YAW){
        control_head_yaw = MIN_HEAD_YAW;
    }

    // Publish the overt attention mode and value saying that mutual gaze is being detected
    overt_attention_mode_msg.state = ATTENTION_SEEKING_STATE;
    overt_attention_mode_msg.value = DETECTING_MUTUAL_GAZE;
    overt_attention_mode_pub.publish(overt_attention_mode_msg);

    // if(move_robot_head(head_topic, control_head_pitch, control_head_yaw, 0.5, debug) != 0){
    //     ROS_WARN("%s: error moving the robot's head to the seeking location", node_name.c_str());
    //     return -1;                                                      // return -1 if the robot's head is not moved to the specified seeking location
    // }

    if(move_robot_head_biological_motion(head_topic, control_head_pitch, control_head_yaw, 1.0, debug) != 0){
        ROS_WARN("%s: error moving the robot's head to the seeking location", node_name.c_str());
        return -1;                                                      // return -1 if the robot's head is not moved to the specified seeking location
    }

    ros::Duration(3).sleep();

    return 1;
}


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
double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI;          // David Vernon ... cast to float
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
    double radians = degrees / ((double) 180.0 / (double) M_PI);        // David Vernon ... cast to float
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
void prompt_and_exit(int status){
    printf("%s: Press any key to exit the program...", node_name.c_str());
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
    printf("%s: Press any key to continue or press X to exit the program...", node_name.c_str());
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
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
    shutdown_requested = true;

    ROS_ERROR("%s: terminated.", node_name.c_str());
    ros::shutdown();
}