/* robotNavigationTestInterface.h - robot navigation unit test interfaceing and library importing that help the unit test ROS Node.
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

#ifndef ROBOT_NAVIGATION_TEST_INTERFACE_H
#define ROBOT_NAVIGATION_TEST_INTERFACE_H

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

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>

// ROS message types
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

// ROS service types  
#include <cssr_system/setGoal.h>

// Google Test
#include <gtest/gtest.h>

// Boost libraries
#include <boost/algorithm/string.hpp>

// OpenCV for basic image handling (optional)
#include <opencv2/opencv.hpp>

// Test-specific constants
#define SOFTWARE_VERSION                "v1.0"
#define INITIALIZATION_INFO_PERIOD      5.0
#define ROS_PACKAGE_NAME                "unit_tests"
#define CSSR_SYSTEM_PACKAGE_NAME        "cssr_system"

// Algorithm identifiers (we define our own to avoid include issues)
#define TEST_BFS_ALGORITHM              0
#define TEST_DIJKSTRA_ALGORITHM         1  
#define TEST_ASTAR_ALGORITHM            2

using namespace std;

// Global test variables
extern std::string testReportFile;
extern std::string testReportPath;
extern std::string testReportPathAndFile;

// Test environment maps (optional)
extern cv::Mat testEnvironmentMap;
extern cv::Mat testConfigurationMap;

// Test configuration flags
extern bool runPathPlanningBfsTest;
extern bool runPathPlanningDijkstraTest;
extern bool runPathPlanningAstarTest;
extern bool runServiceTests;
extern bool runBoundaryTests;
extern bool runConfigurationTests;
extern bool runAlgorithmComparisonTest;

extern std::string nodeName;

// Test scenario file reading functions
std::vector<std::vector<double>> readTestScenariosFromFile(const std::string& filename);
std::vector<std::vector<double>> readBoundaryTestScenariosFromFile(const std::string& filename);
std::vector<std::vector<double>> readServiceTestScenariosFromFile(const std::string& filename);

// Configuration management functions
int readRobotNavigationTestConfiguration(bool* debugMode);
int writeRobotNavigationTestConfiguration(std::string environmentMapFile, 
                                        std::string configurationMapFile, std::string robotTopicsFilename, 
                                        std::string verboseModeInput);
int writeStringToFile(string filename, std::string directory, 
                     std::vector<std::vector<std::string>> content, std::string separator);

// Service and testing utility functions
std::string invokeService(const char* cmd);
bool isServiceAvailable(const std::string& serviceName);
void shutDownHandler(int sig);

// Robot pose management functions
void setRobotPose(double x, double y, double theta);

// Test configuration helper functions
int writeTestConfigurationForAlgorithm(const std::string& algorithm);

// Test execution helper functions  
bool performBasicNavigationTest(double goalX, double goalY, double goalTheta);

// Test result logging functions
void logPathPlanningTestResult(const std::string& testName, const std::string& algorithm, 
                              double startX, double startY, double startTheta,
                              double goalX, double goalY, double goalTheta, bool result);
void logBoundaryTestResult(const std::string& testName, double goalX, double goalY, double goalTheta, bool result);
void logServiceTestResult(const std::string& testName, double goalX, double goalY, double goalTheta, bool result);
void logPerformanceTestResult(const std::string& algorithm, int executionTimeMs, bool pathFound, int waypointCount);

// Utility function for string trimming
inline void trim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// Google Test fixture class
class RobotNavigationUnitTest : public ::testing::Test {
protected:
    void SetUp() override;
    void TearDown() override;
    
    // Helper function declarations only
    bool validateTestEnvironment();
    void waitForSystemReady(double timeout = 5.0);
    bool performBasicNavigationTest(double goalX, double goalY, double goalTheta);
    bool testAlgorithmViaService(const std::string& algorithm, 
                                double startX, double startY, double startTheta,
                                double goalX, double goalY, double goalTheta);
};


#endif // ROBOT_NAVIGATION_TEST_INTERFACE_H