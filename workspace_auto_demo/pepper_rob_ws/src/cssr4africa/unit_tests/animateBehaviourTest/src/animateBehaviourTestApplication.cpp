/*
* animateBehaviourTestApplication.cpp
*
* This code implements a ROS-based test application that uses Google Test
* to validate the animate behavior module, generates structured test reports, 
* and supports continuous testing with user-controlled iterations.
* 
* Implements a ROS node that executes a comprehensive test suite for the animate
* behavior system. The test validates four types of movements:
* - Subtle body joint movements
* - Hand flexing movements
* - Small base rotations around the z-axis
* - Combined movements of all types
* 
* Key features:
* - Continuous test execution capability
* - Automated test report generation
* - Configurable test behavior via external configuration
* - Clean setup and teardown between test runs
* - User-controlled test repetition
* 
* Libraries:
*   - ROS core libraries:
*     - roscpp
*     - ros/package.h
*   - Testing libraries:
*     - gtest/gtest.h
*   - Standard C++ libraries:
*     - iostream
*     - fstream
*     - ctime
*     - string
* 
* Parameters:
*   - argCount: Number of command line arguments
*   - argValues: Array of command line argument strings
* 
* Command-line Parameters:
*   - Standard ROS parameters
*   - Google Test command line options
* 
* Subscribed Topics and Message Types:
*   - None directly (handled by test implementations)
* 
* Published Topics and Message Types:
*   - None directly (handled by test implementations)
* 
* Services Used:
*   - /animateBehaviour/set_activation
*     Used to enable/disable animate behavior during tests
* 
* Input Data Files:
*   - None
*
* Output Data Files:
*   - animateBehaviourTest/data/animateBehaviourTestOutput.dat:Test execution results and timing information
* 
* Configuration Files:
*   - File location: unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini:
*   - File name: animateBehaviourTestConfiguration.ini:
*   - Configuration Settings:
*             armMaximumRange     : 0.2,0.2,0.2,0.35,0.2
*             behaviour           : hands
*             gestureDuration     : 1.0
*             handMaximumRange    : 0.7
*             legMaximumRange     : 0.1,0.1,0.08
*             legRepeatFactor     : 8
*             numPoints           : 100
*             numPointsLeg        : 2
*             platform            : robot
*             robotTopics         : pepperTopics.dat
*             rotMaximumRange     : 0.3
*             selectedRange       : 0.5
*             simulatorTopics     : simulatorTopics.dat
*             verboseMode         : true
*
* Example Instantiation of the Module
*   -roslaunch unit_tests animateBehaviourLaunchTestHarness.launch
*
* Author:  Eyerusalem Mamuye Birhan 
* Email:   ebirhan@andrew.cmu.edu
* Date:    2025-01-10
* Version: 1.0
* 
* Copyright (C) 2023 CSSR4Africa Consortium
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
* Website: www.cssr4africa.org
*/

#include "animateBehaviourTest/animateBehaviourTestInterface.h"

/**
 * @brief Main test driver with continuous test capability
 */
int main(int argCount, char **argValues) {
    std::ofstream testReport;

    ros::init(argCount, argValues, "animateBehaviourTest");
    ros::NodeHandle nh;

   
    nodeName = ros::this_node::getName();
    std::string softwareVersion = "Version: 1.0";
     ROS_INFO("\n%s and %s\n"
         "                  This project is funded by the African Engineering and Technology Network (Afretec)\n"
         "                  This project is funded by the African Engineering and Technology Network (Afretec)\n"
         "                  Inclusive Digital Transformation Research Grant Programme.\n"
         "                  Website: www.cssr4africa.org\n"
         "                  This program comes with ABSOLUTELY NO WARRANTY.", 
         nodeName.c_str(), softwareVersion.c_str());
    
    // Node started up message
    ROS_INFO("%s: start-up.", nodeName.c_str());

   // After node initialization
    heartbeat("start");

   // Add signal handler
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // Termination request
    signal(SIGABRT, signalHandler);  // Abort signal
    signal(SIGQUIT, signalHandler);  // Quit program

    // Initialize ROS and Google Test
    ros::init(argCount, argValues, "animateBehaviourTest");
    ::testing::InitGoogleTest(&argCount, argValues);

    // Construct the full path of the test report file
    std::string basePath = ros::package::getPath(UNIT_TEST_PACKAGE_NAME);
    std::string testReportFile = "animateBehaviourTestOutput.dat";
    std::string testReportPathAndFile = basePath + "/animateBehaviourTest/data/" + testReportFile;
    // ROS_INFO("The test report file is at %s", testReportPathAndFile.c_str());

    // Start ROS spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    bool continueTesting = true;
    
    while (continueTesting && ros::ok()) {
        // Clear the test report file before each test run
        {
            std::ofstream clearReport(testReportPathAndFile, std::ios::out | std::ios::trunc);
            if (!clearReport.is_open()) {
                ROS_ERROR("Failed to clear test report file at %s", testReportPathAndFile.c_str());
                return 1;
            }
            clearReport.close();
        }

        // Reopen file in append mode for the test run
        testReport.open(testReportPathAndFile, std::ios::out | std::ios::app);
        if (!testReport.is_open()) {
            ROS_ERROR("Failed to open test report file at %s", testReportPathAndFile.c_str());
            return 1;
        }

        // Write the header for this test run
        std::time_t now = std::time(nullptr);
        char dateTimeString[100];
        if (std::strftime(dateTimeString, sizeof(dateTimeString), "%Y-%m-%d %H:%M:%S", std::localtime(&now))) {
            testReport << "===============================================================================================\n";
            testReport << "=== New Test Run Started at " << dateTimeString << " ===\n";
            testReport << "===============================================================================================\n\n";
        }
        testReport.flush();
        
        // Set the test report stream for the test class
        AnimateBehaviourRobotTest::setTestReportStream(testReport);

        ROS_INFO("Starting animate behaviour tests...");
        
        // Run all tests
        int result = RUN_ALL_TESTS();

        // Write test run completion
        testReport << "\n===============================================================================================\n";
        testReport << "Test Run Completed with Result: " << (result == 0 ? "PASSED" : "FAILED") << "\n";
        testReport << "===============================================================================================\n\n";
        testReport.flush();

        // Ensure animate behaviour is disabled after tests
        setAnimateBehaviour("disabled");
        // restored the default configuration values 
        restoreDefaultConfiguration();
        
        // Ask if user wants to run tests again
        char response;
        ROS_INFO("Tests completed. Run tests again? (y/n): ");
        std::cin >> response;
        
        // Make sure everything is written before closing
        testReport.flush();
        testReport.close();
        

        continueTesting = (response == 'y' || response == 'Y');
        
        if (continueTesting) {
            ROS_INFO("Preparing for next test run...");
            ros::Duration(2.0).sleep(); 
        } else {
            // stop heartbeat message
            heartbeat("stop");
            ROS_ERROR("%s: Node has been killed/terminated unexpectedly", nodeName.c_str());
            
             // restored the default configuration values 
            restoreDefaultConfiguration();
            performCleanup();
            spinner.stop();
            ros::shutdown();
            _exit(0);  
        }
    }

    // Final cleanup for other exit cases
    performCleanup();
    spinner.stop();
    ros::shutdown();
    return 0;
}

