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

#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>

using namespace boost;
using namespace std;

#define ROS

void headControl(ros::NodeHandle nh);
void rightArmControl(ros::NodeHandle nh);
void leftArmControl(ros::NodeHandle nh);
void legControl(ros::NodeHandle nh);
void wheelsControl(ros::NodeHandle nh);


std::vector<string> extract_tests(string key);
string extract_topic(string set);

void prompt_and_exit(int err);
void prompt_and_continue();
void publish_trajectory(ros::Publisher &pub, std::vector<std::string> &name_vector, trajectory_msgs::JointTrajectoryPoint &t_pnt_msg, ros::Rate &rate);
void publish_velocity(ros::Publisher &pub, geometry_msgs::Twist &msg, ros::Rate &rate);