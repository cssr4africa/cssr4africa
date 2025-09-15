/* robotLocalizationTestInterface.h - Robot Localization Unit Test Interface
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

#ifndef ROBOT_LOCALIZATION_TEST_INTERFACE_H
#define ROBOT_LOCALIZATION_TEST_INTERFACE_H

// Standard C++ libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <cstdio>
#include <ctime>
#include <array>
#include <signal.h>
#include <chrono>
#include <cmath>

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>

// ROS message types
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// ROS service types  
#include <unit_tests/resetPose.h>
#include <unit_tests/setPose.h>

// Google Test
#include <gtest/gtest.h>

// Boost libraries
#include <boost/algorithm/string.hpp>

// OpenCV for ArUco marker generation and validation
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// Test-specific constants
#define SOFTWARE_VERSION                "v1.0"
#define INITIALIZATION_INFO_PERIOD      5.0
#define ROS_PACKAGE_NAME                "unit_tests"
#define CSSR_SYSTEM_PACKAGE_NAME        "cssr_system"

// Test timeout constants
#define DEFAULT_SERVICE_TIMEOUT         10.0
#define POSE_UPDATE_TIMEOUT            5.0
#define MARKER_DETECTION_TIMEOUT       15.0

// Test tolerance constants
#define POSITION_TOLERANCE              0.1  // meters
#define ORIENTATION_TOLERANCE           5.0  // degrees

using namespace std;

// Global test variables
extern std::string testReportFile;
extern std::string testReportPath;
extern std::string testReportPathAndFile;

// Test configuration flags
extern bool runPoseSetTests;
extern bool runPoseResetTests;
extern bool runMarkerDetectionTests;
extern bool runAccuracyTests;
extern bool runServiceTests;
extern bool runStabilityTests;

extern std::string nodeName;

// Current robot pose for tracking
extern geometry_msgs::Pose2D currentRobotPose;
extern bool poseReceived;

// Test scenario structures
struct TestPose {
    double x, y, theta;
    std::string description;
};

struct MarkerTestScenario {
    int markerId;
    double x, y, z;
    std::string description;
};

// Configuration management functions
int readRobotLocalizationTestConfiguration(bool* debugMode);
int writeRobotLocalizationTestConfiguration(std::string landmarkFile, 
                                           std::string topicsFile, 
                                           std::string cameraInfoFile,
                                           std::string verboseModeInput);

// Service and testing utility functions
std::string invokeService(const char* cmd);
bool isServiceAvailable(const std::string& serviceName);
bool isTopicAvailable(const std::string& topicName);
void shutDownHandler(int sig);

// Robot pose management functions
bool setPoseAndVerify(double x, double y, double theta, double timeout = POSE_UPDATE_TIMEOUT);
bool resetPoseAndVerify(double timeout = POSE_UPDATE_TIMEOUT);
bool waitForPoseUpdate(double timeout = POSE_UPDATE_TIMEOUT);

// Test execution helper functions  
bool testPoseAccuracy(const TestPose& expectedPose, double positionTolerance = POSITION_TOLERANCE, 
                     double orientationTolerance = ORIENTATION_TOLERANCE);
bool testServiceResponse(const std::string& serviceName, const std::string& args);

// Test result logging functions
void logPoseTestResult(const std::string& testName, const TestPose& expectedPose, 
                      const TestPose& actualPose, bool result);
void logServiceTestResult(const std::string& testName, const std::string& serviceName, 
                         const std::string& args, bool result);
void logMarkerTestResult(const std::string& testName, int markerId, bool detected, bool result);
void logAccuracyTestResult(const std::string& testName, double positionError, 
                          double orientationError, bool result);

// Utility functions
inline void trim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

double calculateDistance(double x1, double y1, double x2, double y2);
double normalizeDegrees(double degrees);

// ROS callback functions
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

// Google Test fixture class
class RobotLocalizationUnitTest : public ::testing::Test {
protected:
    void SetUp() override;
    void TearDown() override;
    
    // Helper function declarations
    bool validateTestEnvironment();
    void waitForSystemReady(double timeout = 10.0);
    bool performBasicPoseTest(double x, double y, double theta);
    bool performServiceTest(const std::string& serviceName, const std::string& args);
    
    // Test data setup
    std::vector<TestPose> getTestPoses();
    std::vector<MarkerTestScenario> getMarkerScenarios();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    std::ofstream test_report_;
};

#endif // ROBOT_LOCALIZATION_TEST_INTERFACE_H