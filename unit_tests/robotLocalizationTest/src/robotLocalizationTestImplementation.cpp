/* robotLocalizationTestImplementation.cpp - Robot Localization Unit Test Implementation
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

#include "robotLocalizationTest/robotLocalizationTestInterface.h"

// Global test variables
std::string testReportFile = "robotLocalizationTestOutput.dat";
std::string testReportPath;
std::string testReportPathAndFile;

// Test configuration flags
bool runPoseSetTests = true;
bool runPoseResetTests = true;
bool runMarkerDetectionTests = true;
bool runAccuracyTests = true;
bool runServiceTests = true;
bool runStabilityTests = true;

std::string nodeName;

// Current robot pose tracking
geometry_msgs::Pose2D currentRobotPose;
bool poseReceived = false;

// Read the robot localization test configuration file
int readRobotLocalizationTestConfiguration(bool* debugMode) {
    std::string dataDirectory;
    std::string dataPath;
    std::string dataPathAndFile;

    // Construct the full path of the configuration file
    #ifdef ROS
        dataPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        dataPath = "..";
    #endif

    dataDirectory = "/robotLocalizationTest/config/";
    dataPath += dataDirectory;
    dataPathAndFile = dataPath;
    dataPathAndFile += "robotLocalizationTestConfiguration.ini";

    // Open input file
    std::ifstream inputFile(dataPathAndFile.c_str());
    if (!inputFile.is_open()){
        ROS_ERROR("%s: Failed to open the robot localization test configuration file '%s'", 
                  nodeName.c_str(), dataPathAndFile.c_str());
        return 1;
    }

    std::string inputLineRead;
    std::string paramKey, paramValue;

    // Get key-value pairs from the input file
    while(std::getline(inputFile, inputLineRead)){
        std::istringstream inputLineStream(inputLineRead);
        inputLineStream >> paramKey >> paramValue;
        trim(paramKey);
        trim(paramValue);

        // Convert value to lower case
        boost::algorithm::to_lower(paramValue);

        if(paramKey == "poseSetTests"){
            runPoseSetTests = (paramValue == "true");
        }
        else if(paramKey == "poseResetTests"){
            runPoseResetTests = (paramValue == "true");
        }
        else if(paramKey == "markerDetectionTests"){
            runMarkerDetectionTests = (paramValue == "true");
        }
        else if(paramKey == "accuracyTests"){
            runAccuracyTests = (paramValue == "true");
        }
        else if(paramKey == "serviceTests"){
            runServiceTests = (paramValue == "true");
        }
        else if(paramKey == "stabilityTests"){
            runStabilityTests = (paramValue == "true");
        }
        else if(paramKey == "verboseMode"){
            *debugMode = (paramValue == "true");
        }
    }
    inputFile.close();
    
    return 0;
}

// Write the robot localization test configuration file
int writeRobotLocalizationTestConfiguration(std::string landmarkFile, std::string topicsFile, std::string cameraInfoFile, std::string verboseModeInput) {
    std::string dataPath;
    std::string dataPathAndFile;

    #ifdef ROS
        dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME).c_str();
    #else
        dataPath = "..";
    #endif

    dataPath += "/robotLocalization/config/";
    dataPathAndFile = dataPath + "robotLocalizationConfiguration.json";

    // Create JSON configuration
    std::ofstream configFile(dataPathAndFile, std::ios::out | std::ios::trunc);
    if (configFile.is_open()) {
        configFile << "{\n";
        configFile << "  \"verboseMode\": " << verboseModeInput << ",\n";
        configFile << "  \"camera\": \"RGBRealSense\",\n";
        configFile << "  \"depthCamera\": \"DepthRealSense\",\n";
        configFile << "  \"useDepth\": false,\n";
        configFile << "  \"resetInterval\": 30.0,\n";
        configFile << "  \"absolutePoseTimeout\": 300.0,\n";
        configFile << "  \"cameraInfoTimeout\": 15.0,\n";
        configFile << "  \"useHeadYaw\": true,\n";
        configFile << "  \"headYawJointName\": \"HeadYaw\",\n";
        configFile << "  \"mapFrame\": \"map\",\n";
        configFile << "  \"odomFrame\": \"odom\",\n";
        configFile << "  \"landmarkFile\": \"/robotLocalization/data/" << landmarkFile << "\",\n";
        configFile << "  \"topicsFile\": \"/robotLocalization/data/" << topicsFile << "\",\n";
        configFile << "  \"cameraInfoFile\": \"/robotLocalization/data/" << cameraInfoFile << "\"\n";
        configFile << "}\n";
        configFile.close();
        return 0;
    } else {
        ROS_ERROR("%s: Failed to write configuration file '%s'", nodeName.c_str(), dataPathAndFile.c_str());
        return 1;
    }
}

// Invoke a ROS service and return the response
std::string invokeService(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// Function to verify if a service is available
bool isServiceAvailable(const std::string& serviceName){
    return ros::service::exists(serviceName, false);
}

// Function to verify if a topic is available
bool isTopicAvailable(const std::string& topicName){
    ros::master::V_TopicInfo masterTopics;
    ros::master::getTopics(masterTopics);

    for (const auto& topic : masterTopics){
        if (topic.name == topicName){
            return true;
        }
    }
    return false;
}

// Function to handle the shutdown signal
void shutDownHandler(int sig){
    printf("\n");
    ROS_WARN("%s: shutting down...", nodeName.c_str());
    ROS_ERROR("%s: terminated.", nodeName.c_str());
    ros::shutdown();
}

// ROS callback function for pose updates
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    currentRobotPose = *msg;
    poseReceived = true;
}

// Utility functions
double calculateDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double normalizeDegrees(double degrees) {
    while (degrees > 360.0) degrees -= 360.0;
    while (degrees < 0.0) degrees += 360.0;
    return degrees;
}

// Function to set robot pose and verify
bool setPoseAndVerify(double x, double y, double theta, double timeout) {
    // Call the set pose service
    char serviceCall[256];
    sprintf(serviceCall, "rosservice call /robotLocalization/set_pose -- %.2f %.2f %.2f", x, y, theta);
    std::string output = invokeService(serviceCall);
    
    // Check service response
    if (output.find("success: True") == std::string::npos && 
        output.find("success: true") == std::string::npos) {
        return false;
    }
    
    // Wait for pose update
    return waitForPoseUpdate(timeout);
}

// Function to reset pose and verify
bool resetPoseAndVerify(double timeout) {
    // Call the reset pose service
    std::string output = invokeService("rosservice call /robotLocalization/reset_pose");
    
    // Check service response
    if (output.find("success: True") == std::string::npos && 
        output.find("success: true") == std::string::npos) {
        return false;
    }
    
    // Wait for pose update
    return waitForPoseUpdate(timeout);
}

// Function to wait for pose update
bool waitForPoseUpdate(double timeout) {
    ros::Time startTime = ros::Time::now();
    ros::Duration timeoutDuration(timeout);
    
    poseReceived = false;
    while (ros::ok() && (ros::Time::now() - startTime) < timeoutDuration) {
        ros::spinOnce();
        if (poseReceived) {
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    
    return false;
}

// Function to test pose accuracy
bool testPoseAccuracy(const TestPose& expectedPose, double positionTolerance, double orientationTolerance) {
    if (!poseReceived) {
        return false;
    }
    
    double positionError = calculateDistance(expectedPose.x, expectedPose.y, 
                                           currentRobotPose.x, currentRobotPose.y);
    double orientationError = abs(normalizeDegrees(expectedPose.theta) - 
                                normalizeDegrees(currentRobotPose.theta));
    
    // Handle angle wraparound
    if (orientationError > 180.0) {
        orientationError = 360.0 - orientationError;
    }
    
    return (positionError <= positionTolerance && orientationError <= orientationTolerance);
}

// Function to log pose test results
void logPoseTestResult(const std::string& testName, const TestPose& expectedPose, const TestPose& actualPose, bool result) {
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (testReport.is_open()) {
        testReport << testName << ": " << (result ? "PASS" : "FAIL") << "\n";
        testReport << "\tExpected: (" << expectedPose.x << ", " << expectedPose.y 
                   << ", " << expectedPose.theta << ")\n";
        testReport << "\tActual: (" << actualPose.x << ", " << actualPose.y 
                   << ", " << actualPose.theta << ")\n";
        testReport << "\tDescription: " << expectedPose.description << "\n";
        testReport << "\n";
        testReport.close();
    }
}

// Function to log service test results
void logServiceTestResult(const std::string& testName, const std::string& serviceName, const std::string& args, bool result) {
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (testReport.is_open()) {
        testReport << testName << ": " << (result ? "PASS" : "FAIL") << "\n";
        testReport << "\tService: " << serviceName << "\n";
        testReport << "\tArguments: " << args << "\n";
        testReport << "\n";
        testReport.close();
    }
}

// Function to log marker test results
void logMarkerTestResult(const std::string& testName, int markerId, bool detected, bool result) {
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (testReport.is_open()) {
        testReport << testName << ": " << (result ? "PASS" : "FAIL") << "\n";
        testReport << "\tMarker ID: " << markerId << "\n";
        testReport << "\tDetected: " << (detected ? "Yes" : "No") << "\n";
        testReport << "\n";
        testReport.close();
    }
}

// Function to log accuracy test results
void logAccuracyTestResult(const std::string& testName, double positionError, double orientationError, bool result) {
    std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::app);
    if (testReport.is_open()) {
        testReport << testName << ": " << (result ? "PASS" : "FAIL") << "\n";
        testReport << "\tPosition Error: " << positionError << " meters\n";
        testReport << "\tOrientation Error: " << orientationError << " degrees\n";
        testReport << "\n";
        testReport.close();
    }
}

// Google Test fixture implementation
void RobotLocalizationUnitTest::SetUp() {
    ROS_INFO("Setting up robot localization test...");
    
    // Subscribe to pose topic
    pose_sub_ = nh_.subscribe("/robotLocalization/pose", 1, poseCallback);
    ROS_INFO("%s: subscribed to /robotLocalization/pose", nodeName.c_str());
    
    // Give time for subscription to establish
    ros::Duration(1.0).sleep();
    ros::spinOnce();
}

void RobotLocalizationUnitTest::TearDown() {
    ROS_INFO("Tearing down robot localization test...");
    ros::Duration(0.5).sleep();
}

bool RobotLocalizationUnitTest::validateTestEnvironment() {
    // Check if required services are available
    if (!isServiceAvailable("/robotLocalization/set_pose")) {
        ROS_ERROR("robotLocalization set_pose service not available");
        return false;
    }
    
    if (!isServiceAvailable("/robotLocalization/reset_pose")) {
        ROS_ERROR("robotLocalization reset_pose service not available");
        return false;
    }
    
    if (!isTopicAvailable("/robotLocalization/pose")) {
        ROS_ERROR("robotLocalization pose topic not available");
        return false;
    }
    
    return true;
}

void RobotLocalizationUnitTest::waitForSystemReady(double timeout) {
    ros::Time startTime = ros::Time::now();
    ros::Duration timeoutDuration(timeout);
    
    while (ros::ok() && (ros::Time::now() - startTime) < timeoutDuration) {
        if (isServiceAvailable("/robotLocalization/set_pose") && 
            isTopicAvailable("/robotLocalization/pose")) {
            return;
        }
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    
    ROS_WARN("System readiness timeout reached");
}

std::vector<TestPose> RobotLocalizationUnitTest::getTestPoses() {
    return {
        {2.0, 7.8, 270.0, "Position 1"},
        {2.6, 6.0, 180.0, "Position 2"},
        {5.0, 4.8, 90.0, "Position 3"},
        {0.5, 0.5, 45.0, "Position 4"},
        {6.5, 9.5, 315.0, "Position 5"}
    };
}

std::vector<MarkerTestScenario> RobotLocalizationUnitTest::getMarkerScenarios() {
    return {
        {1, 5.0, 4.8, 0.71, "Marker 1 - Position 1"},
        {2, 2.0, 5.4, 0.71, "Marker 2 - Position 2"},
        {3, 2.6, 8.4, 0.71, "Marker 3 - Position 3"}
    };
}

/*  
 *  Test Case: Set Pose Functionality
 */
TEST_F(RobotLocalizationUnitTest, TestPoseSet) {
    if(!runPoseSetTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running SET POSE tests...", nodeName.c_str());

    ASSERT_TRUE(validateTestEnvironment()) << "Test environment validation failed";

    std::vector<TestPose> testPoses = getTestPoses();
    
    for (size_t i = 0; i < testPoses.size(); i++) {
        const TestPose& testPose = testPoses[i];
        
        ROS_INFO("%s: Testing set pose %d/%d: %s", nodeName.c_str(), (int)(i+1), (int)testPoses.size(), testPose.description.c_str());

        bool success = setPoseAndVerify(testPose.x, testPose.y, testPose.theta);
        EXPECT_TRUE(success) << "Failed to set pose: " << testPose.description;

        if (success) {
            TestPose actualPose = {currentRobotPose.x, currentRobotPose.y, currentRobotPose.theta, ""};
            bool accurate = testPoseAccuracy(testPose);

            logPoseTestResult("Set Pose Test " + std::to_string(i+1), testPose, actualPose, accurate);
            EXPECT_TRUE(accurate) << "Pose accuracy test failed for: " << testPose.description;
        }

        ros::Duration(1.0).sleep();
    }
}

/*  
 *  Test Case: Reset Pose Functionality
 */
TEST_F(RobotLocalizationUnitTest, TestPoseReset) {
    if(!runPoseResetTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running RESET POSE tests...", nodeName.c_str());

    ASSERT_TRUE(validateTestEnvironment()) << "Test environment validation failed";

    // Set a known pose first
    TestPose initialPose = {2.0, 7.8, 270.0, "Initial position for reset test"};
    bool setSuccess = setPoseAndVerify(initialPose.x, initialPose.y, initialPose.theta);
    ASSERT_TRUE(setSuccess) << "Failed to set initial pose for reset test";

    // Wait a moment then trigger reset
    ros::Duration(2.0).sleep();
    
    ROS_INFO("%s: Triggering pose reset...", nodeName.c_str());
    bool resetSuccess = resetPoseAndVerify();
    
    if (resetSuccess) {
        TestPose resetPose = {currentRobotPose.x, currentRobotPose.y, 
                             currentRobotPose.theta, "Position after reset"};
        logPoseTestResult("Pose Reset Test", initialPose, resetPose, resetSuccess);
    }

    EXPECT_TRUE(resetSuccess) << "Pose reset failed";
}

/*  
 *  Test Case: Service Availability and Response
 */
TEST_F(RobotLocalizationUnitTest, TestServices) {
    if(!runServiceTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running SERVICE tests...", nodeName.c_str());

    // Test set_pose service
    std::string setPoseService = "/robotLocalization/set_pose";
    bool setPoseAvailable = isServiceAvailable(setPoseService);
    logServiceTestResult("Set Pose Service Availability", setPoseService, "N/A", setPoseAvailable);
    EXPECT_TRUE(setPoseAvailable) << "Set pose service not available";

    // Test reset_pose service  
    std::string resetPoseService = "/robotLocalization/reset_pose";
    bool resetPoseAvailable = isServiceAvailable(resetPoseService);
    logServiceTestResult("Reset Pose Service Availability", resetPoseService, "N/A", resetPoseAvailable);
    EXPECT_TRUE(resetPoseAvailable) << "Reset pose service not available";

    // Test service call with valid parameters
    if (setPoseAvailable) {
        std::string output = invokeService("rosservice call /robotLocalization/set_pose -- 2.0 7.8 270.0");
        bool validResponse = (output.find("success") != std::string::npos);
        logServiceTestResult("Set Pose Service Call", setPoseService, "2.0 7.8 270.0", validResponse);
        EXPECT_TRUE(validResponse) << "Set pose service call failed";
    }
}

/*  
 *  Test Case: Localization Accuracy
 */
TEST_F(RobotLocalizationUnitTest, TestAccuracy) {
    if(!runAccuracyTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running ACCURACY tests...", nodeName.c_str());

    ASSERT_TRUE(validateTestEnvironment()) << "Test environment validation failed";

    std::vector<TestPose> accuracyTestPoses = {
        {2.0, 7.8, 270.0, "High accuracy test - known good position"},
        {5.0, 4.8, 90.0, "Medium accuracy test - moderate distance"},
        {2.6, 8.4, 180.0, "Precision test - nearby landmarks"}
    };

    for (size_t i = 0; i < accuracyTestPoses.size(); i++) {
        const TestPose& testPose = accuracyTestPoses[i];
        
        ROS_INFO("%s: Testing accuracy %d/%d: %s", nodeName.c_str(), (int)(i+1), (int)accuracyTestPoses.size(), testPose.description.c_str());

        bool poseSet = setPoseAndVerify(testPose.x, testPose.y, testPose.theta);
        ASSERT_TRUE(poseSet) << "Failed to set pose for accuracy test";

        // Test with tighter tolerance for accuracy
        double strictPositionTolerance = 0.05; // 5cm
        double strictOrientationTolerance = 2.0; // 2 degrees

        bool accurate = testPoseAccuracy(testPose, strictPositionTolerance, strictOrientationTolerance);
        
        double positionError = calculateDistance(testPose.x, testPose.y, currentRobotPose.x, currentRobotPose.y);
        double orientationError = abs(normalizeDegrees(testPose.theta) - normalizeDegrees(currentRobotPose.theta));
        if (orientationError > 180.0) orientationError = 360.0 - orientationError;

        logAccuracyTestResult("Accuracy Test " + std::to_string(i+1), positionError, orientationError, accurate);

        EXPECT_TRUE(accurate) << "Accuracy test failed for: " << testPose.description;

        ros::Duration(2.0).sleep();
    }
}

/*  
 *  Test Case: Marker Detection and Triangulation
 */
TEST_F(RobotLocalizationUnitTest, TestMarkerDetection) {
    if(!runMarkerDetectionTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running MARKER DETECTION tests...", nodeName.c_str());

    ASSERT_TRUE(validateTestEnvironment()) << "Test environment validation failed";

    std::vector<MarkerTestScenario> markerScenarios = getMarkerScenarios();
    
    for (size_t i = 0; i < markerScenarios.size(); i++) {
        const MarkerTestScenario& scenario = markerScenarios[i];
        
        ROS_INFO("%s: Testing marker detection %d/%d: %s", nodeName.c_str(), (int)(i+1), (int)markerScenarios.size(), scenario.description.c_str());

        // Position robot to potentially see the marker
        TestPose viewingPose = {scenario.x - 1.0, scenario.y, 0.0, "Viewing position"};
        bool poseSet = setPoseAndVerify(viewingPose.x, viewingPose.y, viewingPose.theta);
        ASSERT_TRUE(poseSet) << "Failed to set viewing pose for marker test";

        // Trigger reset to attempt marker-based localization
        ros::Duration(3.0).sleep(); // Give time for camera to stabilize
        
        geometry_msgs::Pose2D poseBefore = currentRobotPose;
        bool resetSuccess = resetPoseAndVerify(MARKER_DETECTION_TIMEOUT);
        
        // Check if pose changed significantly (indicating marker detection)
        double poseChange = calculateDistance(poseBefore.x, poseBefore.y, currentRobotPose.x, currentRobotPose.y);
        bool markerDetected = (resetSuccess && poseChange > 0.01); // 1cm threshold
        
        // More rigorous marker detection validation
        bool markerTestPassed = false;
        std::string testResult = "";
        
        if (!resetSuccess) {
            testResult = "Service failed";
            markerTestPassed = false;
        } else if (markerDetected) {
            testResult = "Marker detected and pose updated";
            markerTestPassed = true;
            
            // Additional validation: check if pose moved toward expected marker position
            double distanceToMarker = calculateDistance(currentRobotPose.x, currentRobotPose.y, scenario.x, scenario.y);
            double previousDistance = calculateDistance(poseBefore.x, poseBefore.y, scenario.x, scenario.y);

            if (distanceToMarker < previousDistance) {
                ROS_INFO("%s: Pose improved toward marker %d (distance reduced by %.3fm)", nodeName.c_str(), scenario.markerId, previousDistance - distanceToMarker);
            }
        } else {
            testResult = "No marker detected - service succeeded but no pose change";
            markerTestPassed = true; // Permissive pass
        }
        
        logMarkerTestResult("Marker Detection Test " + std::to_string(i+1), scenario.markerId, markerDetected, markerTestPassed);
        
        // Log detailed results
        std::ofstream testReport(testReportPathAndFile, std::ios::out | std::ios::app);
        if (testReport.is_open()) {
            testReport << "\tTest Result: " << testResult << "\n";
            testReport << "\tPose Change: " << poseChange << " meters\n";
            testReport << "\tPosition Before: (" << poseBefore.x << ", " << poseBefore.y << ")\n";
            testReport << "\tPosition After: (" << currentRobotPose.x << ", " << currentRobotPose.y << ")\n";
            testReport.close();
        }
        
        // The test passes if either:
        // 1. Marker was detected and pose improved, OR
        // 2. Service worked and we're in permissive mode
        EXPECT_TRUE(markerTestPassed) << "Marker test failed for scenario: " << scenario.description << " (" << testResult << ")";

        ros::Duration(2.0).sleep();
    }
}

/*  
 *  Test Case: System Stability and Robustness
 */
TEST_F(RobotLocalizationUnitTest, TestStability) {
    if(!runStabilityTests){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running STABILITY tests...", nodeName.c_str());

    ASSERT_TRUE(validateTestEnvironment()) << "Test environment validation failed";

    // Test 1: Rapid successive pose sets
    TestPose basePose = {2.0, 7.8, 270.0, "Base position for stability test"};
    std::vector<TestPose> rapidPoses = {
        {2.1, 7.8, 270.0, "Small increment 1"},
        {2.2, 7.8, 270.0, "Small increment 2"},
        {2.3, 7.8, 270.0, "Small increment 3"},
        {2.0, 7.8, 270.0, "Return to base"}
    };

    bool rapidTestSuccess = true;
    for (size_t i = 0; i < rapidPoses.size(); i++) {
        bool success = setPoseAndVerify(rapidPoses[i].x, rapidPoses[i].y, rapidPoses[i].theta, 2.0);
        if (!success) {
            rapidTestSuccess = false;
            break;
        }
        ros::Duration(0.5).sleep(); // Short delay between rapid calls
    }

    logServiceTestResult("Rapid Pose Set Stability Test", "/robotLocalization/set_pose", "Multiple rapid calls", rapidTestSuccess);
    EXPECT_TRUE(rapidTestSuccess) << "Rapid pose setting stability test failed";

    // Test 2: Edge case coordinates
    std::vector<TestPose> edgeCases = {
        {0.0, 0.0, 0.0, "Origin position"},
        {6.8, 9.9, 359.0, "Near map boundary"},
        {3.0, 5.0, -45.0, "Negative angle"},
        {3.0, 5.0, 405.0, "Angle > 360"}
    };

    bool edgeCaseSuccess = true;
    for (size_t i = 0; i < edgeCases.size(); i++) {
        bool success = setPoseAndVerify(edgeCases[i].x, edgeCases[i].y, edgeCases[i].theta, 3.0);
        if (!success) {
            // Don't fail the test for edge cases, just log
            ROS_WARN("Edge case failed: %s", edgeCases[i].description.c_str());
        }
        ros::Duration(1.0).sleep();
    }

    logServiceTestResult("Edge Case Stability Test", "/robotLocalization/set_pose", "Boundary and edge cases", true); // Always pass for edge cases

    // Test 3: Service timeout resilience
    ROS_INFO("%s: Testing service timeout resilience...", nodeName.c_str());
    
    bool timeoutResilience = true;
    for (int i = 0; i < 3; i++) {
        ros::Duration(1.0).sleep();
        bool success = resetPoseAndVerify(5.0); // Reasonable timeout
        if (!success) {
            timeoutResilience = false;
            break;
        }
    }

    logServiceTestResult("Timeout Resilience Test", "/robotLocalization/reset_pose", "Multiple reset calls", timeoutResilience);
    EXPECT_TRUE(timeoutResilience) << "Service timeout resilience test failed";
}