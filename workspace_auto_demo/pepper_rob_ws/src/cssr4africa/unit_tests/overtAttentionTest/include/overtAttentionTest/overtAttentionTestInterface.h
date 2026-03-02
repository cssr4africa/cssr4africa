/* overtAttentionTestInterface.h
*
* Author: Muhammed Danso and Adedayo Akinade
* Date: January 10, 2025
* Version: v1.0
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
#ifndef OVERT_ATTENTION_TEST_INTERFACE_H
#define OVERT_ATTENTION_TEST_INTERFACE_H

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <ctime>
#include <cstdio>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cmath>   
#include <memory>
#include <array>
#include <std_msgs/String.h> 

#include "cssr_system/Status.h"
#include "unit_tests/faceDetection.h"

#define ROS

#define SOFTWARE_VERSION "v1.0"

using namespace boost::algorithm;
using namespace std;

#define CSSR_SYSTEM_PACKAGE_NAME "cssr_system"

#define INITIALIZATION_INFO_PERIOD  5.0
#define OPERATION_INFO_PERIOD       5.0

//  Constants for the status of mutual gaze detection
#define DETECTING_MUTUAL_GAZE       1
#define MUTUAL_GAZE_DETECTED        2
#define MUTUAL_GAZE_NOT_DETECTED    3

extern std::string node_name;                                                   // The name of the node

// The test report file variables
extern std::string test_report_file ;                                         // data filename
extern std::string test_report_path;                                          // data path
extern std::string test_report_path_and_file;                                 // data path and filename

extern bool mutual_gaze_detected;                                             // Stores the status of mutual gaze detection
extern int status;                                                            // Stores the mode status value
extern ros::Publisher speech_pub;

extern bool test_scanning_mode;
extern bool test_social_mode;
extern bool test_seeking_mode;
extern bool test_disabled_mode;
extern bool test_location_mode;

class OvertAttentionUnitTest : public ::testing::Test {
    protected:
        std::ofstream test_report;
        // Run before each test
        virtual void SetUp() {
            // Open the test report file in append mode
            test_report.open(test_report_path_and_file, std::ios::out | std::ios::app); 
            if (!test_report.is_open()) {
                ROS_ERROR("%s: failed to open test report file: '%s'", node_name.c_str(), test_report_path_and_file.c_str());
            }
        }

        // Run after each test
        virtual void TearDown() {
            // Close the test report file
            if (test_report.is_open()) {
                test_report.close();
            }
        }

        // Log the test result to the test report file
        void logTestResult(const std::string& testName, bool result) {
            if (test_report.is_open()) {
                test_report << testName;
                test_report << "\n\tResult        : " << (result ? "PASSED" : "FAILED") << "\n\n";
                test_report.flush();
            } else {
                ROS_ERROR("%s: test report file is not open.", node_name.c_str());
            }
        }
};

/*  
 *  Function to read the overt attention test configuration file
 *
 *  The function reads the attention test configuration file and sets the variables for the attention tests
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 * 
 *  @return
 *      0 if the attention test configuration file is read successfully
 *      1 if the attention test configuration file is not read successfully
 */
int read_overt_attention_test_configuration(string* platform, bool* debug);

/*  Function to invoke the overt attention mode service and return the response from the service
 *  
 * @param:
 *     cmd: the service command to run
 * 
 * @return
 *    result: the response from the service
 */
std::string invoke_service(const char* cmd);

/*  
 *   Function to handle the shutdown signal
 *   Cancels all active goals and shuts down ROS
 *  @param:
 *      sig: the signal
 *  @return:
 *      None
 */
void shut_down_handler(int sig);

/*
 *   Callback function for the overt attention mode data received from the /overtAttention/mode topic
 *   The function receives the overt attention mode data and determines the status
 */
void mode_data_received(const cssr_system::Status& mode_msg);

/*
 *   Callback function for the face detection data received from the /face_detection/data topic
 *   The function receives the face detection data and determines mutual gaze
 */
void face_detection_data_received(const unit_tests::faceDetection& data_msg);

/* 
 *   Function to verify if a topic is available
 * @param:
 *   topic_name: string to store the topic name
 * 
 * @return:
 *  boolean indicating if the topic is available
 */
bool is_topic_available(std::string topic_name);

#endif // OVERT_ATTENTION_TEST_INTERFACE_H