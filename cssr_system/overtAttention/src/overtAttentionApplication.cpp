/* overtAttentionApplication.cpp
*
* <detailed functional description>
* This module module equips the robot with the ability to direct its gaze toward salient features in its environment 
* or focus on specific locations, facilitating socially and contextually appropriate behaviors. 
* This capability is crucial for enhancing the robot’s ability to interact effectively with people and adapt to dynamic environments.
*
* The module operates in five distinct modes, each tailored to a specific context. 
*   - Social mode is activated during social interactions, allowing the robot to focus on human faces, and voices. 
*     In this mode, the robot prioritizes social cues to maintain engagement and responsiveness. 
*
*   - Scanning mode, on the other hand, is used when the robot is not engaged in social interaction. 
*     In this state, the robot scans its surroundings for potential interaction opportunities, 
*     focusing on people and objects of interest while giving higher priority to faces. 
*     The robot periodically shifts its focus to new areas, ensuring comprehensive environmental coverage.
*
*   - In location mode, the robot gazes at a specific target in its environment. 
*     If the robot’s head cannot achieve the required pose to fixate on the target, 
*     the robot’s base rotates to realign its head and body. 
*
*   - Seeking mode enables the robot to establish mutual gaze with a nearby person by searching for a face looking directly at it. 
*     If this process is unsuccessful within a given timeframe, the robot returns either a success or failure status. 
*
*   - Lastly, in disabled mode, the robot’s head remains centered and stationary, effectively deactivating the attention mechanism.
*
* To determine the focus of attention, the module generates two types of saliency maps. 
*   - A social saliency map leverages data from face detection and sound localization to identify socially significant features. 
*   - A general saliency map, on the other hand, uses information-theoretic models to identify visually conspicuous elements 
*     in the robot’s environment. These maps form the basis for the robot’s attentional behavior across different modes.
*
* In scanning mode, three key processes work together to enhance attentional dynamics. 
*   - First, a winner-take-all (WTA) mechanism identifies a single focus of attention from the saliency map using a selective tuning model. 
*   - Second, an Inhibition-of-Return (IOR) mechanism ensures that previously attended locations are deprioritized, encouraging exploration of new areas. Third, a habituation process gradually reduces the salience of the current focus, ensuring that attention does not remain fixated on a single point for an extended period.
*
* The robot’s gaze is directed by publishing control commands to the headYaw and headPitch joints, 
* which align the head toward the selected focus of attention. 
* For aural attention, the robot adjusts its headYaw angle based on the angle of arrival of the sound. 
* Calibration parameters ensure accurate mapping between visual offsets in the image and the corresponding head joint angles. 
* When the required headYaw rotation exceeds a predefined threshold, the module coordinates the movement of the robot’s base 
* and head to maintain focus while realigning the head and torso.
*
* The module’s functionality is supported by four key inputs. 
* Data from the face detection and sound detection nodes inform the saliency map in social mode. 
* An RGB image from the robot’s camera is used to compute the saliency map in scanning mode, 
* while the robot’s current pose is utilized for attending to specific locations. 
* These inputs enable the module to adapt its behavior dynamically based on environmental conditions.
*
* The module provides four outputs to facilitate its operation. 
* First, it publishes control commands to the robot’s headYaw and headPitch joints, 
* as well as to the wheels and angular velocity when adjusting the robot’s pose. 
* Second, it generates an RGB image visualizing the saliency function and the current focus of attention, 
* which can be displayed in verbose mode for debugging purposes. 
* Third, the module continuously publishes the current active mode to the /overtAttention/mode topic, 
* enabling other system components to monitor the robot’s attentional state. 
* Finally, the module updates actuator topic names based on configuration files specific to either the physical robot or a simulation environment.
*
* The module’s operation is managed through dedicated ROS services. The module advertises services to allow the selection 
* of operational modes, such as social, scanning, or location mode. 
*
* For ease of analysis, the module can also operate in verbose mode, where published data is printed to the terminal, 
* and output images are displayed in OpenCV windows.
*
...
* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h, cv_bridge.h

...
* Parameters
*
* Command-line Parameters
*
* The attention mode to set the attention system to
* The location in the world to pay attention to in x, y, z coordinates
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform                    simulator
* camera                      FrontCamera
* realignmentThreshold        5
* xOffsetToHeadYaw            25
* yOffsetToHeadPitch          20
* simulatorTopics             simulatorTopics.dat
* robotTopics                 pepperTopics.dat
* verboseMode                 true

...
* Subscribed Topics and Message Types
*
* /faceDetection/direction              faceDetection.msg     
* /robotLocalization/pose               sensor_msgs::JointState
* /soundDetection/data                  std_msgs::Float64    
* /naoqi_driver/camera/front/image_raw  sensor_msgs::ImageConstPtr       
...
* Published Topics and Message Types
* 
* /pepper_dcm/Head_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /cmd_vel                                                      geometry_msgs/Twist
* /overtAttention/mode                                          Status.msg

...
* Invoked Services
* 
* None

...
* Services advertised
* 
* /overtAttention/set_mode                                         
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* None
...
* Configuration Files
*
* overtAttentionConfiguration.ini
...
* Example Instantiation of the Module
*
* rosrun cssr_system overtAttention
...
*
* The clients can call the service by providing the attention mode and the location to pay attention to in the world.
* The service will execute the attention mode selected and attend to the location provided if mode being set is location mode.
* AN example of calling the service is shown below:
* ----- rosservice call /overAttention/set_mode -- location 3.0 2.0 1.0
* This will set the attention mode to location and the location to pay attention to is (3.0, 2.0, 1.0)
*
...
*
* Author: Muhammed Danso and Adedayo Akinade, Carnegie Mellon University Africa
* Email: mdanso@andrew.cmu.edu, aakinade@andrew.cmu.edu
* Date: January 10, 2025
* Version: v1.0
*
*/

# include "overtAttention/overtAttentionInterface.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "overtAttention", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    node_name = ros::this_node::getName();                                      // Get the name of the node

    int scanning_state_value = 0;                                               // Stores the scanning state value

    int attention_execution_status = 0;                                         // Stores the status of the attention execution

    // Initialize the random number generator to be able to randomly select faces when multiple faces are present in social mode
    unsigned random_seed = std::chrono::system_clock::now().time_since_epoch().count();
    random_generator = std::mt19937(random_seed);

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                   // The signal handler for the interrupt signal    

    std::string copyright_message = node_name + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t This project is funded by the African Engineering and Technology Network (Afretec)\n\t\t\t\t\t\t Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                  // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());                                // Print startup message

    // Read the configuration file
    int config_file_read = 0;
    config_file_read = read_configuration_file(&implementation_platform, &camera_type, &realignment_threshold, &x_offset_to_head_yaw, &y_offset_to_head_pitch, &simulator_topics, &robot_topics, &topics_filename, &social_attention_mode, &verbose_mode);
    
    // Check if the configuration file was read successfully
    if(config_file_read == 1){
        ROS_ERROR("%s: error reading the configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }  

    // Extract the topic for the camera
    std::string camera_topic;                                                   // stores the camera topic
    if(extract_topic(camera_type, topics_filename, &camera_topic) != 0){
        ROS_ERROR("%s: error extracting the camera topic.", node_name.c_str());
        shut_down_handler(0);
        return -1;                                                              // return -1 if the camera topic is not extracted successfully
    }

    // Set the image parameters based on the camera type
    set_image_parameters(implementation_platform, camera_type, &horizontal_fov, &vertical_fov, &camera_image_width, &camera_image_height);

    // Create an image transport subscriber
    image_transport::ImageTransport it(nh);
    ROS_INFO("%s: subscribing to %s...", node_name.c_str(), camera_topic.c_str());
    image_transport::Subscriber sub = it.subscribe(camera_topic, 1, front_camera_message_received);
    ROS_INFO("%s: subscribed to %s.", node_name.c_str(), camera_topic.c_str());
    
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

    // Create a subscriber object for faceDetection data
    std::string face_detection_topic;
    if(extract_topic("FaceDetection", topics_filename, &face_detection_topic)){
        ROS_ERROR("%s: error extracting the face detection topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s topic...", node_name.c_str(), face_detection_topic.c_str());
    while(ros::ok() && !is_topic_available(face_detection_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), face_detection_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber face_detection_subscriber = nh.subscribe(face_detection_topic, 1, &face_detection_data_received);
    ROS_INFO("%s: subscribed to %s topic.", node_name.c_str(), face_detection_topic.c_str());

    // Create a subscriber object for sound localization data
    std::string sound_localization_topic;
    if(extract_topic("SoundLocalization", topics_filename, &sound_localization_topic)){
        ROS_ERROR("%s: error extracting the sound localization topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s topic...", node_name.c_str(), sound_localization_topic.c_str());
    while(ros::ok() && !is_topic_available(sound_localization_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), sound_localization_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber sound_localization_subscriber = nh.subscribe(sound_localization_topic, 1, &sound_localization_data_received);
    ROS_INFO("%s: subscribed to %s topic.", node_name.c_str(), sound_localization_topic.c_str());
    
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
    ROS_INFO("%s: advertised the %s service", node_name.c_str(), set_mode_service_name.c_str());

    // Advertise the /overtAttention/mode topic
    std::string engagement_mode_topic = "/overtAttention/mode";
    ROS_INFO("%s: advertising the %s topic...", node_name.c_str(), engagement_mode_topic.c_str());
    while(ros::ok() && !is_topic_available(engagement_mode_topic)){
        overt_attention_mode_pub = nh.advertise<cssr_system::Status>(engagement_mode_topic, 1);
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: advertised the %s topic", node_name.c_str(), engagement_mode_topic.c_str());

    // Print the node ready message after initialization complete
    ROS_INFO("%s: initialization complete...", node_name.c_str());

    // Extract the topic for the head
    std::string head_topic;     // stores the head topic
    if(extract_topic("Head", topics_filename, &head_topic) == 0){
        // Set to head to horizontal looking forward
        if(move_robot_head_biological_motion(head_topic, DEFAULT_HEAD_PITCH, DEFAULT_HEAD_YAW, 1.0, verbose_mode) != 0){
            ROS_ERROR("%s: error setting the head to the horizontal looking forward pose.", node_name.c_str());
            shut_down_handler(0);
            return -1;                                                                // return -1 if the head cannot be set to the horizontal looking forward pose
        }
        node_initialized = true;                                                     // Set the node initialized flag to true
    }

    // Main loop
    while(ros::ok()){   
        ROS_INFO_THROTTLE(OPERATION_INFO_PERIOD, "%s: running...", node_name.c_str());  // Print a message every 10 seconds
        ros::spinOnce();                                                        // Check for new messages on the topics
        // ros::Duration(1).sleep();                                               // Sleep for 1 second

        /* Execute the attention mode selected in the request */
        switch(attention_mode){
            case ATTENTION_MODE_DISABLED:                                       // Disabled attention mode
                // Publish the disabled attention mode status
                overt_attention_mode_msg.state = ATTENTION_DISABLED_STATE;
                overt_attention_mode_msg.value = ATTENTION_MODE_DISABLED;
                overt_attention_mode_pub.publish(overt_attention_mode_msg);

                if(!disabled_once){                                             // If the disabled mode has not been called at least once, set the previous time to the current time
                    // Set head to center pose looking forward
                    if(move_robot_head_biological_motion(head_topic, DEFAULT_HEAD_PITCH, DEFAULT_HEAD_YAW, 1.0, verbose_mode) != 0){
                        ROS_ERROR("%s: error setting the head to the horizontal looking forward pose in disabled mode.", node_name.c_str());
                    }
                    disabled_once = true;
                }
                break;
            case ATTENTION_MODE_SOCIAL:                                         // Social attention mode
                // Publish the social attention mode status
                overt_attention_mode_msg.state = ATTENTION_SOCIAL_STATE;
                overt_attention_mode_msg.value = ATTENTION_MODE_DEFAULT;
                overt_attention_mode_pub.publish(overt_attention_mode_msg);

                // Call the social attention function
                if(!social_attention_done){                                     // If the social attention has not been done from the last time messages arrived
                    attention_execution_status = social_attention(topics_filename, realignment_threshold, attention_velocity_publisher, social_attention_mode, verbose_mode);
                    social_attention_done = true;                               // Set the social attention done flag to true                       
                }
                break;
            case ATTENTION_MODE_SCANNING:                                       // Scanning attention mode
                {
                    // Publish the scanning attention mode status
                    overt_attention_mode_msg.state = ATTENTION_SCANNING_STATE;
                    scanning_state_value = DETECTING_MUTUAL_GAZE;
                    if(mutual_gaze_detected){
                        scanning_state_value = MUTUAL_GAZE_DETECTED;
                        mutual_gaze_detected = false;
                        face_detected = false;
                    }
                    overt_attention_mode_msg.value = scanning_state_value;
                    overt_attention_mode_pub.publish(overt_attention_mode_msg);
                    
                    // Compute the saliency features and get the center of the most salient region
                    int centre_x = 0;
                    int centre_y = 0;
                    int saliency_features_status = compute_saliency_features(camera_image, &centre_x, &centre_y, verbose_mode);

                    if(saliency_features_status != 0){
                        ROS_ERROR("Error computing the saliency features\n");
                        break;
                    }
                    
                    // Get the head angles from the pixel coordinates
                    AngleChange angle_change = get_angles_from_pixel(centre_x, centre_y, camera_image_width, camera_image_height, horizontal_fov, vertical_fov);
                    double control_pitch = radians(angle_change.delta_pitch) + head_joint_states[0];
                    double control_yaw = radians(angle_change.delta_yaw) + head_joint_states[1];

                    // Store the winning location
                    previous_locations.push_back(std::make_tuple(control_yaw, control_pitch, 1));

                    // Call the scanning attention function
                    attention_execution_status = scanning_attention(control_yaw, control_pitch, topics_filename, attention_velocity_publisher, verbose_mode);
                    break;
                }
            case ATTENTION_MODE_SEEKING:                                        // Scanning attention mode
                // Call the seeking attention function
                // current_secs = ros::Time::now().toSec();
                // if(!seek_once){                                                 // If the seek mode has not been called at least once, set the previous time to the current time
                //     previous_secs = current_secs;
                //     seeking_completed = false;
                // }

                if(mutual_gaze_detected){                                       // If mutual gaze is detected, publish the information and break
                    // Publish the overt attention mode and value saying that mutual gaze is detected
                    overt_attention_mode_msg.state = ATTENTION_SEEKING_STATE;
                    overt_attention_mode_msg.value = MUTUAL_GAZE_DETECTED;
                    overt_attention_mode_pub.publish(overt_attention_mode_msg);
                    seeking_completed = true;
                    break;
                }

                // if(current_secs - previous_secs <= ENGAGEMENT_TIMEOUT){
                if(!seeking_completed){                                         // If seeking is not completed, call the seeking attention function
                    // Publish the overt attention mode and value saying that mutual gaze is being detected
                    overt_attention_mode_msg.state = ATTENTION_SEEKING_STATE;
                    overt_attention_mode_msg.value = DETECTING_MUTUAL_GAZE;
                    overt_attention_mode_pub.publish(overt_attention_mode_msg);

                    // Call the seeking attention function
                    attention_execution_status = seeking_attention(topics_filename, realignment_threshold, attention_velocity_publisher, overt_attention_mode_pub, verbose_mode);
                    seek_once = true;                                           // Set the seek once flag to true
                    break;
                }
                
                // If seeking is completed, publish the information and break
                overt_attention_mode_msg.state = ATTENTION_SEEKING_STATE;
                overt_attention_mode_msg.value = MUTUAL_GAZE_NOT_DETECTED;
                overt_attention_mode_pub.publish(overt_attention_mode_msg);        
                break;
            case ATTENTION_MODE_LOCATION:                                       // Location attention mode
                // Publish the location attention mode status
                overt_attention_mode_msg.state = ATTENTION_LOCATION_STATE;
                overt_attention_mode_msg.value = ATTENTION_MODE_DEFAULT;
                overt_attention_mode_pub.publish(overt_attention_mode_msg);
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
