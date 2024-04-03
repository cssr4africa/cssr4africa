#ifndef ACTUATORTEST_H
#define ACTUATORTEST_H

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <thread>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cmath>
#include <signal.h>  

#define ROS
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ControlClient;
typedef boost::shared_ptr<ControlClient> ControlClientPtr;
using namespace boost::algorithm;

std::string extractTopic(std::string key);
std::string extractMode();
std::vector<std::string> extractTests(std::string set);
void promptAndExit(int status);
void promptAndContinue();
void signalHandler(int signum);

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                    const std::string& positionName, std::vector<double> positions);
void executeTestsSequentially(const std::vector<std::string>& testNames, ros::NodeHandle& nh);
void executeTestsInParallel(const std::vector<std::string>& testNames, ros::NodeHandle& nh);

std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, std::vector<double> minPosition, std::vector<std::vector<double>> velocity);

void head(ros::NodeHandle& nh);
void rArm(ros::NodeHandle& nh);
void lArm(ros::NodeHandle& nh);
void rHand(ros::NodeHandle& nh);
void lHand(ros::NodeHandle& nh);
void leg(ros::NodeHandle& nh);
void wheels(ros::NodeHandle& nh);

#endif // ACTUATORTEST_H