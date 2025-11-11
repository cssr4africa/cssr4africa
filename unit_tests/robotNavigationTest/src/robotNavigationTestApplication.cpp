/* robotNavigationTestApplication.cpp - robot navigation unit test ROS Node definition
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

/* robotNavigationTestApplication.cpp - Application code to run the robot Navigation Unit test.
*
* This module is responsible for running the tests on the robot navigation module.
* The tests are run using Google Test and the results are written to a file. 
* The module tests the path planning algorithms (BFS, Dijkstra, A*) via service calls,
* navigation service functionality, boundary conditions, and configuration management.
*
* Libraries
*       Standard libraries
-           std::string, std::vector, std::fstream
*       ROS libraries
-           ros/ros.h, ros/package.h, cssr_system/setGoal.h
*
* Parameters
*       Command-line Parameters
*           None
*       Configuration File Parameters
*           Key                   |     Value 
*           --------------------- |     -------------------
*           pathPlanningBfs       |     true
*           pathPlanningDijkstra  |     true
*           pathPlanningAstar     |     true
*           serviceTests          |     true
*           boundaryTests         |     true
*           configurationTests    |     true
*           verboseMode           |     true
*
* Subscribed Topics and Message Types
*       None
*
* Published Topics and Message Types
*       None
*
* Services Used
*       /robotNavigation/set_goal                                      cssr_system/setGoal
*
* Services Advertised and Message Types
*       None
*
* Input Data Files
*       astarAlgorithmInput.dat
*       bfsAlgorithmInput.dat
*       boundaryTestInput.dat
*       dijkstraAlgorithmInput.dat
*       navigationServiceInput.dat
*       pepperTopics.dat
*
* Output Data Files
*       robotNavigationTestOutput.dat
*
* Configuration Files
*       robotNavigationTestConfiguration.ini
*
* Example Instantiation of the Module
*       rosrun unit_tests robotNavigationTest
*
* The launch file for the robot navigation unit tests is robotNavigationLaunchTestHarness.launch.
*       roslaunch unit_tests robotNavigationLaunchTestHarness.launch
*
* Author:   Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email:    bgirmash@andrew.cmu.edu
* Date:     January 10, 2025
* Version:  v1.0
*
*/

#include "robotNavigationTest/robotNavigationTestInterface.h"
#include <thread>

int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "robotNavigationTest", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ::testing::InitGoogleTest(&argc, argv);

    // Register the signal handler
    signal(SIGINT, shutDownHandler);

    nodeName = ros::this_node::getName();
    if (nodeName[0] == '/') {
        nodeName = nodeName.substr(1);
    }

    std::string copyrightMessage = nodeName + ":  " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\tWebsite: www.cssr4africa.org "
                                    "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyrightMessage.c_str());
    ROS_INFO("%s: start-up.", nodeName.c_str());
    

    // Set the contents of the robot navigation test configuration
    string environmentMapFileConfig            = "scenarioOneEnvironmentMap.dat";
    string configurationMapFileConfig          = "scenarioOneConfigMap.dat";
    string robotTopicsFilename                 = "pepperTopics.dat";
    string verboseModeInput                    = "true";
    bool verboseMode = false;

    // Read the robot navigation test configuration file
    if(readRobotNavigationTestConfiguration(&verboseMode) != 0){
        ROS_ERROR("%s: error reading the robot navigation test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 0;
    }
    
    // Check if the /robotNavigation/set_goal service is available
    std::string setGoalServiceName = "/robotNavigation/set_goal";
    while(ros::ok() && !isServiceAvailable(setGoalServiceName)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s service to be available...", nodeName.c_str(), setGoalServiceName.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s service available.", nodeName.c_str(), setGoalServiceName.c_str());

    // Check if the /robotLocalization/reset_pose service is available
    std::string resetPoseServiceName = "/robotLocalization/set_pose";
    if(!isServiceAvailable(resetPoseServiceName)){
        ROS_WARN("%s: %s service not available - pose setting may not work properly", nodeName.c_str(), resetPoseServiceName.c_str());
    } else {
        ROS_INFO("%s: %s service available.", nodeName.c_str(), resetPoseServiceName.c_str());
    }

    // Construct the full path of the test report file
    #ifdef ROS
        testReportPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        testReportPath = "..";
    #endif

    // set configuration path
    testReportPath += "/robotNavigationTest/data/";
    testReportPathAndFile = testReportPath;
    testReportPathAndFile += testReportFile;

    ROS_INFO("%s: Creating the test report file '%s'...", nodeName.c_str(), testReportPathAndFile.c_str());

    // Clear the test report file and write the header before running any tests
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::trunc);
    if (testReport.is_open()) {
        std::time_t now = std::time(nullptr);
        char dateTimeString[100];
        if (std::strftime(dateTimeString, sizeof(dateTimeString), "%Y-%m-%d %H:%M:%S", std::localtime(&now))) {
            testReport << "Robot Navigation Test Report: " << "\n";
            testReport << "==========================================\n";
            testReport << "Date: " << dateTimeString << "\n";
            testReport << "Environment Map: " << environmentMapFileConfig << "\n";
            testReport << "Configuration Map: " << configurationMapFileConfig << "\n";
            testReport << "Robot Topics: " << robotTopicsFilename << "\n";
            testReport << "Verbose Mode: " << (verboseMode ? "true" : "false") << "\n";
            testReport << "Test Method: Service-based testing\n\n";
        }
        ROS_INFO("%s: Test report file '%s' created.", nodeName.c_str(), testReportPathAndFile.c_str());
        testReport.close();
    } else {
        ROS_ERROR("%s: Failed to create test report file '%s'", nodeName.c_str(), testReportPathAndFile.c_str());
        shutDownHandler(0);
        return 1;
    }

    verboseModeInput = verboseMode ? "true" : "false";
    
    // Write robot navigation test configuration file for current test run
    if(writeRobotNavigationTestConfiguration(environmentMapFileConfig, configurationMapFileConfig, robotTopicsFilename, verboseModeInput) != 0){
        ROS_ERROR("%s: error writing the robot navigation test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 1;
    }

    // Try to load test environment maps for visualization (if needed)
    std::string packagePath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME);
    if (!packagePath.empty()) {
        std::string envMapPath = packagePath + "/robotNavigation/data/" + environmentMapFileConfig;
        std::string configMapPath = packagePath + "/robotNavigation/data/" + configurationMapFileConfig;
        
        testEnvironmentMap = cv::imread(envMapPath, cv::IMREAD_GRAYSCALE);
        testConfigurationMap = cv::imread(configMapPath, cv::IMREAD_GRAYSCALE);
        
        if (!testEnvironmentMap.empty()) {
            ROS_INFO("%s: Loaded environment map from '%s'", nodeName.c_str(), envMapPath.c_str());
        }
        
        if (!testConfigurationMap.empty()) {
            ROS_INFO("%s: Loaded configuration map from '%s'", nodeName.c_str(), configMapPath.c_str());
        }
    }

    // Print test configuration summary
    ROS_INFO("%s: Test Configuration:", nodeName.c_str());
    ROS_INFO("%s: Path Planning BFS Test: %s", nodeName.c_str(), runPathPlanningBfsTest ? "enabled" : "disabled");
    ROS_INFO("%s: Path Planning Dijkstra Test: %s", nodeName.c_str(), runPathPlanningDijkstraTest ? "enabled" : "disabled");
    ROS_INFO("%s: Path Planning A* Test: %s", nodeName.c_str(), runPathPlanningAstarTest ? "enabled" : "disabled");
    ROS_INFO("%s: Service Tests: %s", nodeName.c_str(), runServiceTests ? "enabled" : "disabled");
    ROS_INFO("%s: Boundary Tests: %s", nodeName.c_str(), runBoundaryTests ? "enabled" : "disabled");

    // Print the node ready message after initialization complete
    ROS_INFO("%s: ready.", nodeName.c_str());
    
    // Start a separate thread for ROS spinning with heartbeat
    std::thread spin_thread([&]() {
        ros::Rate rate(10);  // 10 Hz
        while(ros::ok()) {
            ROS_INFO_THROTTLE(10, "%s: running.", nodeName.c_str());
            ros::spinOnce();
            rate.sleep();
        }
    });

    // Run all the tests
    int result = 0;
    result = RUN_ALL_TESTS();

    // Stop the spinner thread
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    // Write test summary to report
    std::ofstream summaryReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (summaryReport.is_open()) {
        summaryReport << "\n==========================================\n";
        summaryReport << "Test Execution Summary\n";
        summaryReport << "==========================================\n";
        if (result == 0) {
            summaryReport << "Overall Result: ALL TESTS PASSED\n";
            ROS_INFO("%s: All tests completed successfully!", nodeName.c_str());
        } else {
            summaryReport << "Overall Result: SOME TESTS FAILED\n";
            summaryReport << "Failed test count: " << result << "\n";
            ROS_WARN("%s: Some tests failed. Check the test report for details.", nodeName.c_str());
        }
        summaryReport << "Test report location: " << testReportPathAndFile << "\n";
        summaryReport.close();
    }

    // Rewrite the configuration file to its default values
    environmentMapFileConfig    = "scenarioOneEnvironmentMap.dat";
    configurationMapFileConfig  = "scenarioOneConfigMap.dat";
    verboseModeInput            = "false";
    if(writeRobotNavigationTestConfiguration(environmentMapFileConfig, configurationMapFileConfig, robotTopicsFilename, verboseModeInput) != 0){
        ROS_ERROR("%s: error writing the robot navigation test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 1;
    }
    
    
    // Shutdown the ROS spinner and return the test result
    shutDownHandler(0);
    return result;

    
}
