/* robotNavigationDriver.cpp - a driver software that simulates robot localization
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

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <signal.h>
#include <iomanip>
#include <angles/angles.h>
#include "unit_tests/setPose.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#define ROS
#define SOFTWARE_VERSION            "v1.0"
#define INITIALIZATION_INFO_PERIOD 5.0
#define OPERATION_INFO_PERIOD 10.0
#define PI       3.14159

using namespace std;
using namespace boost::algorithm;

double odomX           = 0.0;
double odomY           = 0.0;
double odomTheta       = 0.0;

double currentX        = 0.0;
double currentY        = 0.0;
double currentTheta    = 0.0;

double adjustmentX     = 0.0;
double adjustmentY     = 0.0;
double adjustmentTheta = 0.0;

double initialX        = 2.0;
double initialY        = 7.8;
double initialTheta    = 270.0;

ros::Publisher posePub;

string nodeName;

bool nodeInitialized = false;
string topicsFilename;
string platform;
bool verboseMode;

void odometryMessageReceived(const nav_msgs::Odometry &msg)
{
   bool debug = true;

   double x, y;

   odomX = msg.pose.pose.position.x;
   odomY = msg.pose.pose.position.y;
   odomTheta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

   /* translation of origin */
   x = odomX + adjustmentX - initialX;
   y = odomY + adjustmentY - initialY;

   /* rotation about origin */
   currentX = x * cos(adjustmentTheta) + y * -sin(adjustmentTheta);
   currentY = x * sin(adjustmentTheta) + y * cos(adjustmentTheta);

   currentX += initialX;
   currentY += initialY;

   currentTheta = odomTheta + adjustmentTheta;

   /* check to ensure theta is still in the range -PI to +PI */
   if (currentTheta < -PI)
      currentTheta += 2 * PI;
   else if (currentTheta > PI)
      currentTheta -= 2 * PI;

   // Prepare message for publishing
    geometry_msgs::Pose2D poseMsg;
    poseMsg.x = currentX;
    poseMsg.y = currentY;
    poseMsg.theta = angles::to_degrees(currentTheta); // Convert to degrees

    // Publish the Pose2D message
    posePub.publish(poseMsg);   
}

bool setPose(unit_tests::setPose::Request &req, unit_tests::setPose::Response &res)
{
    // Store the desired initial pose
    initialX = req.x;
    initialY = req.y;
    initialTheta = angles::from_degrees(req.theta);

    // Wait for the first odometry message
    ros::Rate rate(10);
   
    // **Ensure odometry data is up-to-date**
    ros::spinOnce();

    // Calculate adjustments between initial pose and current odometry
    adjustmentX = initialX - odomX;
    adjustmentY = initialY - odomY;
    adjustmentTheta = initialTheta - odomTheta;

    res.success = true;

    return true;
}

/*  
 *  Function to read the robot navigation test input file
 */
int readRobotNavigationTestConfiguration(string* topicsFilename, bool *debug) {
    std::string dataDirectory;
    std::string dataPath;
    std::string dataPathAndFile;

    // Construct the full path of the configuration file
    #ifdef ROS
        dataPath = ros::package::getPath("unit_tests").c_str();
    #else
        dataPath = "..";
    #endif

    // set configuration path
    dataDirectory = "/robotNavigationTest/config/";
    dataPath += dataDirectory;
    dataPathAndFile = dataPath;
    dataPathAndFile += "robotNavigationTestConfiguration.ini";

    // Open input file
    std::ifstream inputFile(dataPathAndFile.c_str());
    if (!inputFile.is_open()){
        ROS_ERROR("%s: Failed to open the robot navigation test configuration file '%s'", nodeName.c_str(), dataPathAndFile.c_str());
        return 1;
    }

    std::string inputLineRead;
    std::string paramKey, paramValue;

    // Set the platform value to robot by default
    platform = "robot";

    // Get key-value pairs from the input file
    while(std::getline(inputFile, inputLineRead)){
        std::istringstream inputLineStream(inputLineRead);
        inputLineStream >> paramKey >> paramValue;
        trim(paramKey);
        trim(paramValue);

        // convert the key and value to lower case
        boost::algorithm::to_lower(paramValue);

        if(paramKey == "verboseMode"){
            if(paramValue == "true"){
                *debug = true;
            }
            else{
                *debug = false;
            }
        }
    }
    inputFile.close();

    if(platform == "robot"){
        *topicsFilename = "pepperTopics.dat";
    }
    else if(platform == "simulator"){
        *topicsFilename = "simulatorTopics.dat";
    }

    return 0;    
}

/* 
 *   Function to verify if a topic is available
 */
bool isTopicAvailable(std::string topicName){
    bool topicAvailable = false;
    ros::master::V_TopicInfo masterTopics;
    ros::master::getTopics(masterTopics);

    // Iterate through the topics to check if the topic is available
    for (const auto& topic : masterTopics){
        if (topic.name == topicName){
            topicAvailable = true;
            break;
        }
    }

    return topicAvailable;
}

/*  
 *   Function to extract the topic from the topics file
 */
int extractTopic(string key, string topicFileName, string* topicValue){
    bool debug = false;
    
    std::string topicPath;
    std::string topicPathAndFile;

    // Construct the full path of the topic file
    #ifdef ROS
        topicPath = ros::package::getPath("unit_tests").c_str();
    #else
        topicPath = "..";
    #endif

    // set topic path    
    topicPath += "/robotNavigationTest/data/";
    topicPathAndFile = topicPath;
    topicPathAndFile += topicFileName;

    // Open topic file
    std::ifstream topicIf(topicPathAndFile.c_str());
    if (!topicIf.is_open()){
        ROS_ERROR("%s: Unable to open the topic file %s", nodeName.c_str(), topicPathAndFile.c_str());
        return 1;
    }

    std::string topicLineRead;
    // Get key-value pairs from the topic file
    while(std::getline(topicIf, topicLineRead)){
        std::istringstream iss(topicLineRead);
        std::string paramKey;
        std::string paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        if (paramKey == key) {
            *topicValue = paramValue;
            break;
        }
    }
    topicIf.close();

    // verify the topicValue is not empty
    if (*topicValue == ""){
        ROS_ERROR("%s: unable to find a valid topic for '%s'. Please check the topics file:\n\t\t\t\t\t\t '%s'.", nodeName.c_str(), key.c_str(), topicPathAndFile.c_str());
        return 1;
    }
    return 0;
}

/*  
 *   Function to handle the shutdown signal
 */
void shutDownHandler(int sig) {
    printf("\n");
    ROS_WARN("%s: shutting down...", nodeName.c_str());
    ROS_ERROR("%s: terminated.", nodeName.c_str());
    ros::shutdown();
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "robotNavigationTestDriver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    nodeName = ros::this_node::getName();
    if (nodeName[0] == '/') {
        nodeName = nodeName.substr(1);}
    // Register the signal handler
    signal(SIGINT, shutDownHandler);

    std::string copyrightMessage = nodeName + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\tWebsite: www.cssr4africa.org "
                                    "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyrightMessage.c_str());
    ROS_INFO("%s: start-up.", nodeName.c_str());

    // Read the robot navigation test configuration file
    if(readRobotNavigationTestConfiguration(&topicsFilename, &verboseMode) != 0){
        ROS_ERROR("%s: error reading the robot navigation test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 0;
    }

    // Extract the odometry topic from the topics file and subscribe to it
    string odometryTopic;
    if(extractTopic("Odometry", topicsFilename, &odometryTopic)){
        ROS_ERROR("%s: error extracting the odometry topic", nodeName.c_str());
        shutDownHandler(0);
        return 0;
    }
    ROS_INFO("%s: subscribing to %s...", nodeName.c_str(), odometryTopic.c_str());
    while(ros::ok() && !isTopicAvailable(odometryTopic)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", nodeName.c_str(), odometryTopic.c_str());
        ros::Duration(1).sleep();
    }
    ros::Subscriber odomSub = nh.subscribe(odometryTopic, 1, &odometryMessageReceived);
    ROS_INFO("%s: subscribed to %s.", nodeName.c_str(), odometryTopic.c_str());
    
    // Extract the robot pose topic from the topics file and advertise it
    std::string robotPoseTopic;
    if(extractTopic("RobotPose", topicsFilename, &robotPoseTopic)){
        ROS_ERROR("%s: error extracting the robot pose topic.", nodeName.c_str());
        shutDownHandler(0);
        return 0;
    }
    ROS_INFO("%s: advertising the %s topic...", nodeName.c_str(), robotPoseTopic.c_str());
    posePub = nh.advertise<geometry_msgs::Pose2D>(robotPoseTopic, 1);
    ROS_INFO("%s: %s topic ready", nodeName.c_str(), robotPoseTopic.c_str());

    string resetPoseServiceName = "/robotLocalization/set_pose";
    ROS_INFO("%s: advertising the %s service...", nodeName.c_str(), resetPoseServiceName.c_str());
    ros::ServiceServer resetPoseService = nh.advertiseService(resetPoseServiceName, setPose);
    ROS_INFO("%s: %s service ready", nodeName.c_str(), resetPoseServiceName.c_str());

    // Get the initial pose from the command line or the parameter server
    if(argc > 1){
        initialX = std::stod(argv[1]);
        initialY = std::stod(argv[2]);
        initialTheta = std::stod(argv[3]);
    }
    else{
        if (!nh.getParam("initial_robot_x", initialX)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_x', using default value: %.2f", nodeName.c_str(), initialX);
        }
        if (!nh.getParam("initial_robot_y", initialY)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_y', using default value: %.2f", nodeName.c_str(), initialY);
        }
        if (!nh.getParam("initial_robot_theta", initialTheta)) {
            ROS_WARN("%s: failed to get parameter 'initial_robot_theta', using default value: %.2f", nodeName.c_str(), initialTheta);
        }
    }   
    initialTheta = angles::from_degrees(initialTheta);   

    ros::Duration(1).sleep();
    ros::spinOnce();

    adjustmentX = initialX - odomX;
    adjustmentY = initialY - odomY;
    adjustmentTheta = initialTheta - odomTheta; 

    ROS_INFO("%s: driver ready", nodeName.c_str()); 

    while(ros::ok()){
        ROS_INFO_THROTTLE(OPERATION_INFO_PERIOD, "%s: running...", nodeName.c_str());
        ros::spinOnce();
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

    return 0;
}