#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <thread>
#include <fstream>
#include <string>
#include <unordered_map>
#include <boost/algorithm/string.hpp>
#include <ctime>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <filesystem>
#ifdef PEPPER_ROBOT
#include <naoqi_driver/AudioCustomMsg.h>
#endif // DEBUG

using namespace boost;
using namespace std;
using TestFunction = std::function<void(ros::NodeHandle&)>;
extern bool output;
extern int timeDuration;

#define ROS

void backSonar(ros::NodeHandle nh);
void frontSonar(ros::NodeHandle nh);
void frontCamera(ros::NodeHandle nh);
void bottomCamera(ros::NodeHandle nh);
void depthCamera(ros::NodeHandle nh);
void stereoCamera(ros::NodeHandle nh);
void laserSensor(ros::NodeHandle nh);
void jointState(ros::NodeHandle nh);
void odom(ros::NodeHandle nh);
void imu(ros::NodeHandle nh);
void speech(ros::NodeHandle nh);

/* Call back functions executed when a sensor data arrived */
void backSonarMessageReceived(const sensor_msgs::Range& msg);
void frontSonarMessageReceived(const sensor_msgs::Range& msg);
void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void stereoCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg);
void jointStateMessageReceived(const sensor_msgs::JointState& msg);
void odomMessageReceived(const nav_msgs::Odometry& msg);
void imuMessageReceived(const sensor_msgs::Imu& msg);

#ifdef PEPPER_ROBOT
void microphone(ros::NodeHandle nh);
void microphoneMessageReceived(const naoqi_driver::AudioCustomMsg& msg);
#endif // DEBUG

std::vector<string> extractTests(string key);
string extractTopic(string set);   
std::string extractMode();
void writeWavHeader(std::ofstream &file, int sampleRate, int numSamples);
void playAndDeleteFile();
void initializeOutputFile(std::ofstream& out_of, const std::string& path);
std::string getOutputFilePath();
std::string getCurrentTime();
void finalizeOutputFile(std::ofstream& out_of, const std::string& path);
void executeTestsSequentially(const std::vector<std::string>& testNames, ros::NodeHandle& nh);
void executeTestsInParallel(const std::vector<std::string>& testNames, ros::NodeHandle& nh);
void switchMicrophoneChannel();

void promptAndExit(int err);
void promptAndContinue();

#endif // SENSOR_TEST_H