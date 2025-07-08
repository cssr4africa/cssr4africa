/* robotLocalizationTestApplication.cpp - Robot Localization Unit Test Application
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

/* robotLocalizationTestApplication.cpp - Application code to run the Robot Localization Unit tests.
*
* This module is responsible for running comprehensive tests on the robot localization
* module. The tests are run using Google Test and the results are written to a file.
* The module tests pose setting, pose reset, marker detection, accuracy, service
* functionality, and system stability.
*
* Libraries
*       Standard libraries
-           std::string, std::vector, std::fstream, std::chrono
*       ROS libraries
-           ros/ros.h, ros/package.h, geometry_msgs/Pose2D.h
*       OpenCV libraries
-           opencv2/opencv.hpp, opencv2/aruco.hpp
*
* Parameters
*       Command-line Parameters
*           None
*           
*       Configuration File Parameters
*           Key                      | Value 
*           ------------------------ | -------------------
*           poseSetTests             | true
*           poseResetTests           | true
*           markerDetectionTests     | true
*           accuracyTests            | true
*           serviceTests             | true
*           stabilityTests           | true
*           verboseMode              | true
*
* Subscribed Topics and Message Types
*       /robotLocalization/pose                 geometry_msgs/Pose2D
*       /naoqi_driver/odom                      nav_msgs/Odometry
*
* Published Topics and Message Types
*       None
*
* Services Used
*       /robotLocalization/set_pose             cssr_system/setPose
*       /robotLocalization/reset_pose           cssr_system/resetPose
*
* Services Advertised and Message Types
*       None
*
* Input Data Files
*       arucoLandmarks.json (test landmarks)
*       pepperTopics.dat
*       cameraInfo.json
*
* Output Data Files
*       robotLocalizationTestOutput.dat
*
* Configuration Files
*       robotLocalizationTestConfiguration.ini
*
* Example Instantiation of the Module
*       rosrun unit_tests robotLocalizationTest
*
* The launch file for the robot localization unit tests is robotLocalizationTestLaunchTestHarness.launch.
*       roslaunch unit_tests robotLocalizationTestLaunchTestHarness.launch
*
* Author:   Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email:    ioj@andrew.cmu.edu
* Date:     June 25, 2025
* Version:  v1.0
*
*/

#include "robotLocalizationTest/robotLocalizationTestInterface.h"

int main(int argc, char **argv) {
    // Initialize ROS and Google Test
    ros::init(argc, argv, "robotLocalizationTest", ros::init_options::NoSigintHandler);
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

    // Set the contents of the robot localization test configuration
    string landmarkFileConfig              = "arucoLandmarks.json";
    string topicsFileConfig                = "pepperTopics.dat";
    string cameraInfoFileConfig            = "cameraInfo.json";
    string verboseModeInput                = "true";
    bool verboseMode = false;

    // Read the robot localization test configuration file
    if(readRobotLocalizationTestConfiguration(&verboseMode) != 0){
        ROS_ERROR("%s: error reading the robot localization test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 0;
    }
    
    // Check if the /robotLocalization/set_pose service is available
    std::string setPoseServiceName = "/robotLocalization/set_pose";
    while(ros::ok() && !isServiceAvailable(setPoseServiceName)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s service to be available...", 
                         nodeName.c_str(), setPoseServiceName.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s service available.", nodeName.c_str(), setPoseServiceName.c_str());

    // Check if the /robotLocalization/reset_pose service is available
    std::string resetPoseServiceName = "/robotLocalization/reset_pose";
    while(ros::ok() && !isServiceAvailable(resetPoseServiceName)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s service to be available...", 
                         nodeName.c_str(), resetPoseServiceName.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s service available.", nodeName.c_str(), resetPoseServiceName.c_str());

    // Check if the /robotLocalization/pose topic is available
    std::string poseTopicName = "/robotLocalization/pose";
    while(ros::ok() && !isTopicAvailable(poseTopicName)){
        ROS_WARN_THROTTLE(INITIALIZATION_INFO_PERIOD, "%s: waiting for %s topic to be available...", 
                         nodeName.c_str(), poseTopicName.c_str());
        ros::Duration(1).sleep();
    }
    ROS_INFO("%s: %s topic available.", nodeName.c_str(), poseTopicName.c_str());

    // Construct the full path of the test report file
    #ifdef ROS
        testReportPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        testReportPath = "..";
    #endif

    // set configuration path
    testReportPath += "/robotLocalizationTest/data/";
    testReportPathAndFile = testReportPath;
    testReportPathAndFile += testReportFile;

    ROS_INFO("%s: Creating the test report file '%s'...", nodeName.c_str(), testReportPathAndFile.c_str());

    // Clear the test report file and write the header before running any tests
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::trunc);
    if (testReport.is_open()) {
        std::time_t now = std::time(nullptr);
        char dateTimeString[100];
        if (std::strftime(dateTimeString, sizeof(dateTimeString), "%Y-%m-%d %H:%M:%S", std::localtime(&now))) {
            testReport << "Robot Localization Test Report\n";
            testReport << "==========================================\n";
            testReport << "Date: " << dateTimeString << "\n";
            testReport << "Landmark File: " << landmarkFileConfig << "\n";
            testReport << "Topics File: " << topicsFileConfig << "\n";
            testReport << "Camera Info File: " << cameraInfoFileConfig << "\n";
            testReport << "Verbose Mode: " << (verboseMode ? "true" : "false") << "\n";
            testReport << "Test Method: Service-based testing with pose verification\n\n";
        }
        ROS_INFO("%s: Test report file '%s' created.", nodeName.c_str(), testReportPathAndFile.c_str());
        testReport.close();
    } else {
        ROS_ERROR("%s: Failed to create test report file '%s'", nodeName.c_str(), testReportPathAndFile.c_str());
        shutDownHandler(0);
        return 1;
    }

    verboseModeInput = verboseMode ? "true" : "false";
    
    // Write robot localization test configuration file for current test run
    if(writeRobotLocalizationTestConfiguration(landmarkFileConfig, topicsFileConfig, 
                                              cameraInfoFileConfig, verboseModeInput) != 0){
        ROS_ERROR("%s: error writing the robot localization test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 1;
    }

    // Print test configuration summary
    ROS_INFO("%s: Test Configuration:", nodeName.c_str());
    ROS_INFO("%s: Set Pose Tests: %s", nodeName.c_str(), runPoseSetTests ? "enabled" : "disabled");
    ROS_INFO("%s: Reset Pose Tests: %s", nodeName.c_str(), runPoseResetTests ? "enabled" : "disabled");
    ROS_INFO("%s: Marker Detection Tests: %s", nodeName.c_str(), runMarkerDetectionTests ? "enabled" : "disabled");
    ROS_INFO("%s: Accuracy Tests: %s", nodeName.c_str(), runAccuracyTests ? "enabled" : "disabled");
    ROS_INFO("%s: Service Tests: %s", nodeName.c_str(), runServiceTests ? "enabled" : "disabled");
    ROS_INFO("%s: Stability Tests: %s", nodeName.c_str(), runStabilityTests ? "enabled" : "disabled");

    // Print the node ready message after initialization complete
    ROS_INFO("%s: ready.", nodeName.c_str());

    // Start the ROS spinner and run the tests
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set up periodic heartbeat timer (every 10 seconds)
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(10.0), 
        [&](const ros::TimerEvent&) {
            ROS_INFO("%s: running.", nodeName.c_str());
        });

    // Run all the tests
    int result = RUN_ALL_TESTS();

    // Get detailed test results for accurate reporting
    const testing::UnitTest* unit_test = testing::UnitTest::GetInstance();
    int total_test_count = unit_test->total_test_count();
    int failed_test_count = unit_test->failed_test_count();
    int passed_test_count = total_test_count - failed_test_count;

    // Write test summary to report
    std::ofstream summaryReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (summaryReport.is_open()) {
        summaryReport << "\n==========================================\n";
        summaryReport << "Test Execution Summary\n";
        summaryReport << "==========================================\n";
        summaryReport << "Total Tests: " << total_test_count << "\n";
        summaryReport << "Passed Tests: " << passed_test_count << "\n";
        summaryReport << "Failed Tests: " << failed_test_count << "\n";
        
        if (failed_test_count == 0) {
            summaryReport << "Overall Result: ALL TESTS PASSED\n";
            ROS_INFO("%s: All tests completed successfully!", nodeName.c_str());
        } else {
            summaryReport << "Overall Result: SOME TESTS FAILED\n";
            summaryReport << "Failed test count: " << failed_test_count << "\n";
            ROS_WARN("%s: %d out of %d tests failed. Check the test report for details.", 
                    nodeName.c_str(), failed_test_count, total_test_count);
        }
        summaryReport << "Test report location: " << testReportPathAndFile << "\n";
        summaryReport.close();
    } else {
        ROS_ERROR("%s: Failed to write test summary to report file '%s'", 
                  nodeName.c_str(), testReportPathAndFile.c_str());
    }

    // Rewrite the configuration file to its default values
    landmarkFileConfig     = "arucoLandmarks.json";
    topicsFileConfig       = "pepperTopics.dat";
    cameraInfoFileConfig   = "cameraInfo.json";
    verboseModeInput       = "false";
    if(writeRobotLocalizationTestConfiguration(landmarkFileConfig, topicsFileConfig, 
                                              cameraInfoFileConfig, verboseModeInput) != 0){
        ROS_ERROR("%s: error writing the robot localization test configuration file.", nodeName.c_str());
        shutDownHandler(0);
        return 1;
    }

    // Shutdown the ROS spinner and return the test result
    spinner.stop();

    if (failed_test_count == 0) {
        ROS_INFO("%s: Test suite completed successfully. All tests passed.", nodeName.c_str());
    } else {
        ROS_WARN("%s: Test suite completed with %d failures out of %d total tests.", 
                nodeName.c_str(), failed_test_count, total_test_count);
    }

    ROS_INFO("%s: Complete. Shutting down", nodeName.c_str());
    ros::shutdown();
    return failed_test_count;
}