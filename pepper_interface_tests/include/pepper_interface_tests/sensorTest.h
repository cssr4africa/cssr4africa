#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>

#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

using namespace boost;
using namespace std;

#define ROS

/* Call back functions executed when a sensor data arrived */
void jointStateCallBack(const sensor_msgs::JointState& state); 
void imageCallBack(const sensor_msgs::ImageConstPtr& msg);


void backSonar(ros::NodeHandle nh);
void frontSonar(ros::NodeHandle nh);
void frontCamera(ros::NodeHandle nh);
void bottomCamera(ros::NodeHandle nh);
void depthCamera(ros::NodeHandle nh);
void laserSensor(ros::NodeHandle nh);

void backSonarMessageReceived(const sensor_msgs::Range& msg);
void frontSonarMessageReceived(const sensor_msgs::Range& msg);
void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg);


std::vector<string> extract_tests(string key);
string extract_topic(string set);

void prompt_and_exit(int err);
void prompt_and_continue();