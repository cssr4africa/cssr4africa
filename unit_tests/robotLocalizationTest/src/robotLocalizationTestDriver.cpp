/* robotLocalizationTestDriver.cpp - Simple driver for testing
*
* Author:   Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email:    ioj@andrew.cmu.edu
* Date:     June 25, 2025
* Version:  v1.0
*
* Copyright (C) 2025 CSSR4Africa Consortium
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
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <signal.h>
#include "unit_tests/setPose.h"
#include "unit_tests/resetPose.h"

double currentX = 4.4;
double currentY = 7.8;  
double currentTheta = 270.0;

ros::Publisher posePub;
std::string nodeName;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Simple driver - just use current pose
    geometry_msgs::Pose2D poseMsg;
    poseMsg.x = currentX;
    poseMsg.y = currentY;
    poseMsg.theta = currentTheta;
    posePub.publish(poseMsg);
}

bool setPoseService(unit_tests::setPose::Request& req, unit_tests::setPose::Response& res) {
    currentX = req.x;
    currentY = req.y;
    currentTheta = req.theta;
    
    // Publish updated pose
    geometry_msgs::Pose2D poseMsg;
    poseMsg.x = currentX;
    poseMsg.y = currentY;
    poseMsg.theta = currentTheta;
    posePub.publish(poseMsg);
    
    res.success = true;
    return true;
}

bool resetPoseService(unit_tests::resetPose::Request& req, unit_tests::resetPose::Response& res) {
    // Simple reset - just return current pose as success
    // In a real implementation, this would trigger marker-based localization
    res.success = true;
    
    // Publish current pose to simulate reset
    geometry_msgs::Pose2D poseMsg;
    poseMsg.x = currentX;
    poseMsg.y = currentY;
    poseMsg.theta = currentTheta;
    posePub.publish(poseMsg);
    
    return true;
}

void shutDownHandler(int sig) {
    ROS_WARN("%s: shutting down...", nodeName.c_str());
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotLocalizationTestDriver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    nodeName = ros::this_node::getName();
    if (nodeName[0] == '/') nodeName = nodeName.substr(1);
    
    signal(SIGINT, shutDownHandler);
    
    ROS_INFO("%s: Robot Localization Test Driver v1.0", nodeName.c_str());
    ROS_INFO("%s: startup.", nodeName.c_str());
    
    // Advertise pose topic
    posePub = nh.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 1);
    
    // Subscribe to odometry for simulation
    ros::Subscriber odomSub = nh.subscribe("/naoqi_driver/odom", 1, odometryCallback);
    
    // Advertise set pose service
    ros::ServiceServer setPoseServ = nh.advertiseService("/robotLocalization/set_pose", setPoseService);
    
    // Advertise reset pose service
    ros::ServiceServer resetPoseServ = nh.advertiseService("/robotLocalization/reset_pose", resetPoseService);
    
    ROS_INFO("%s: driver ready", nodeName.c_str());
    
    // Publish initial pose and setup timer
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
        geometry_msgs::Pose2D poseMsg;
        poseMsg.x = currentX;
        poseMsg.y = currentY;
        poseMsg.theta = currentTheta;
        posePub.publish(poseMsg);
    });
    
    ros::spin();
    return 0;
}