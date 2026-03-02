/* overtAttentionTestDriver.cpp
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

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>                        // Include for publishing Float64 messages
#include "unit_tests/faceDetection.h"
#include <random>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <signal.h>
#include <boost/algorithm/string.hpp>

#define ROS

#define PI       3.14159

#define SOFTWARE_VERSION            "v1.0"

#define INITIALIZATION_INFO_PERIOD 5.0
#define OPERATION_INFO_PERIOD 10.0

#define FACE_PUBLISH_FREQUENCY          1.0
#define SOUND_PUBLISH_FREQUENCY         5.0

using namespace std;
using namespace boost::algorithm;

double odom_x           = 0.0;
double odom_y           = 0.0;
double odom_theta       = 0.0;

double current_x        = 0.0;
double current_y        = 0.0;
double current_theta    = 0.0;

double adjustment_x     = 0.0;
double adjustment_y     = 0.0;
double adjustment_theta = 0.0;

double initial_x        = 0.0;
double initial_y        = 0.0;
double initial_theta    = 0.0;

ros::Publisher pose_pub;

string node_name;

void odometry_message_received(const nav_msgs::Odometry &msg)
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

   // Prepare message for publishing
    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = current_x;
    pose_msg.y = current_y;
    pose_msg.theta = angles::to_degrees(current_theta); // Convert to degrees

    // Publish the Pose2D message
    pose_pub.publish(pose_msg);   
}

/*  
 *  Function to read the overt attention test input file
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
int read_overt_attention_test_configuration(string* platform, string* topics_filename, bool *debug) {
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
    data_directory = "/overtAttentionTest/config/";
    data_path += data_directory;
    data_path_and_file = data_path;
    data_path_and_file += "overtAttentionTestConfiguration.ini";

    // Open input file
    std::ifstream input_file(data_path_and_file.c_str());
    if (!input_file.is_open()){
        ROS_ERROR("%s: unable to open the configuration file %s\n", node_name.c_str(), data_path_and_file.c_str());
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

        return 0;
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
    topic_path += "/overtAttentionTest/data/";
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

    // Shutdown the node
    ROS_ERROR("%s: terminated.", node_name.c_str());
    ros::shutdown();
}

int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "overtAttentionTestDriver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // Register the signal handler
    signal(SIGINT, shut_down_handler);                                   // The signal handler for the interrupt signal    
    
    node_name = ros::this_node::getName();

    std::string copyright_message = node_name + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\t\t\t       This project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\t\t\t       Inclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\t\t\t       Website: www.cssr4africa.org "
                                    "\n\t\t\t\t\t\t       This program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                                      // Print the copyright message

    ROS_INFO("%s: startup.", node_name.c_str());

    string implementation_platform;
    string topics_filename;
    bool verbose_mode;
    
    if(read_overt_attention_test_configuration(&implementation_platform, &topics_filename, &verbose_mode) != 0){
        ROS_ERROR("%s: error reading the overt attention test configuration file.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }

    // Declare the publishers
    ros::Publisher sound_direction_pub;
    ros::Publisher face_detection_pub;
    
    // Extract the odometry topic from the topics file and subscribe to it
    string odometry_topic;
    if(extract_topic("Odometry", topics_filename, &odometry_topic)){
        ROS_ERROR("%s: error extracting the odometry topic", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s...", node_name.c_str(), odometry_topic.c_str());
    while(ros::ok() && !is_topic_available(odometry_topic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", node_name.c_str(), odometry_topic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber odom_sub = nh.subscribe(odometry_topic, 1, &odometry_message_received);
    ROS_INFO("%s: subscribed to %s.", node_name.c_str(), odometry_topic.c_str());

    // Extract the robot pose topic from the topics file and advertise it
    std::string robot_pose_topic;
    if(extract_topic("RobotPose", topics_filename, &robot_pose_topic)){
        ROS_ERROR("%s: error extracting the robot pose topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: advertising the %s topic...", node_name.c_str(), robot_pose_topic.c_str());
    pose_pub = nh.advertise<geometry_msgs::Pose2D>(robot_pose_topic, 1);
    ROS_INFO("%s: %s topic ready", node_name.c_str(), robot_pose_topic.c_str());

    // Initialize the publisher for sound direction
    std::string sound_detection_topic;
    if(extract_topic("soundLocalization", topics_filename, &sound_detection_topic)){
        ROS_ERROR("%s: error extracting the sound localization topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: advertising the %s topic...", node_name.c_str(), sound_detection_topic.c_str());
    sound_direction_pub = nh.advertise<std_msgs::Float64>(sound_detection_topic, 10);
    ROS_INFO("%s: %s topic ready", node_name.c_str(), sound_detection_topic.c_str());
  
    // Initialize the publisher for face detections and mutual gaze
    std::string face_detection_topic;
    if(extract_topic("faceDetection", topics_filename, &face_detection_topic)){
        ROS_ERROR("%s: error extracting the face detection topic.", node_name.c_str());
        shut_down_handler(0);
        return 0;
    }
    ROS_INFO("%s: advertising the %s topic...", node_name.c_str(), face_detection_topic.c_str());
    face_detection_pub = nh.advertise<unit_tests::faceDetection>(face_detection_topic, 1);
    ROS_INFO("%s: %s topic ready", node_name.c_str(), face_detection_topic.c_str());

    // Get the initial pose from the command line or the parameter server
    if(argc > 1){
        initial_x = std::stod(argv[1]);
        initial_y = std::stod(argv[2]);
        initial_theta = std::stod(argv[3]);
    }
    else{
        if (!nh.getParam("initial_robot_x", initial_x)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_x', using default value: %.2f", node_name.c_str(), initial_x);
        }
        if (!nh.getParam("initial_robot_y", initial_y)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_y', using default value: %.2f", node_name.c_str(), initial_y);
        }
        if (!nh.getParam("initial_robot_theta", initial_theta)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_theta', using default value: %.2f", node_name.c_str(), initial_theta);
        }
    }   

    initial_theta = angles::from_degrees(initial_theta);   

    ros::Duration(1).sleep();
    ros::spinOnce();

    adjustment_x = initial_x - odom_x;
    adjustment_y = initial_y - odom_y;
    adjustment_theta = initial_theta - odom_theta; 

    std::default_random_engine generator;
    std::poisson_distribution<int> face_distribution(3);
    std::poisson_distribution<int> sound_distribution(0.5);
    std::uniform_real_distribution<double> sound_angle_distribution(-70.0, 70.0);

    ROS_INFO("%s: drivers ready", node_name.c_str()); 

    double current_time = ros::Time::now().toSec();
    double last_face_publish_time = ros::Time::now().toSec();
    double last_sound_publish_time = ros::Time::now().toSec();

    while(ros::ok()){
        ROS_INFO_THROTTLE(OPERATION_INFO_PERIOD, "%s: running...", node_name.c_str());  // Print a message every 10 seconds
        ros::spinOnce();

        current_time = ros::Time::now().toSec();

        int number_of_faces = face_distribution(generator);
        // std::cout << "faces : " << number_of_faces << std::endl;
        
        int number_of_sound = sound_distribution(generator);
        // std::cout << "sound : " << number_of_sound << std::endl;

        geometry_msgs::Point centroid;
        unit_tests::faceDetection face_detection_msg;
        
        // Providing a seed value
        srand((unsigned) time(NULL));

        if (number_of_faces > 0)
        {
            for (size_t i = 0; i < number_of_faces; i++)
            {
                // if(i > 1){
                //     break;
                // }
                centroid.x = rand() % 640;
                // set centroid.x to 320 if i is 0 and 0 if i is 1 --- COmment out later
                // centroid.x = (i == 0) ? 20 : 620;

                centroid.y = (rand() % 240) + 240;
                // set centroid.y to a fixed value of 360 ---- Comment out later
                // centroid.y = 360;

                centroid.z = rand() % 6;
                // set centroid.z to 2 if i is 0 and 5 if i is 1 --- Comment out later
                // centroid.z = (i == 0) ? 5 : 2;
                

                // ROS_INFO("%s: face centroid: x = %.2f, y = %.2f, z = %.2f", node_name.c_str(), centroid.x, centroid.y, centroid.z);
                
                face_detection_msg.centroids.push_back(centroid);

                double prob = ((double)rand()) / INT_MAX;
                bool gaze = (prob > 0.7) ? true : false;

                face_detection_msg.mutualGaze.push_back(gaze);
            }

            // Publish the face detection message every FACE_PUBLISH_FREQUENCY seconds
            if(current_time - last_face_publish_time >= FACE_PUBLISH_FREQUENCY){
                face_detection_pub.publish(face_detection_msg);
                last_face_publish_time = current_time;
            }
        }
        
        if (number_of_sound > 0)
        {
            double angle = sound_angle_distribution(generator);
            // std::cout << "  > " << angle << std::endl;
            std_msgs::Float64 sound_direction_msg;
            sound_direction_msg.data = angle;
            
            // Publish the sound direction message every SOUND_PUBLISH_FREQUENCY seconds
            if(current_time - last_sound_publish_time >= SOUND_PUBLISH_FREQUENCY){
                sound_direction_pub.publish(sound_direction_msg);
                last_sound_publish_time = current_time;
            }
        }    
    }

    // Clean up the parameter server
    if(nh.hasParam("initial_robot_x")){
        nh.deleteParam("initial_robot_x");
    }
    if(nh.hasParam("initial_robot_y")){
        nh.deleteParam("initial_robot_y");
    }
    if(nh.hasParam("initial_robot_theta")){
        nh.deleteParam("initial_robot_theta");
    }

    return 1;
}