/* gestureExecutionTestInterface.cpp
*
* Author: Adedayo Akinade
* Date: December 16, 2024
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

#ifndef GESTURE_EXECUTION_TEST_INTERFACE_H
#define GESTURE_EXECUTION_TEST_INTERFACE_H

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service_client.h>
#include <ros/master.h>
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
#include <signal.h>
#include <std_msgs/String.h>

#define ROS


using namespace boost::algorithm;
using namespace std;

#define CSSR_SYSTEM_PACKAGE_NAME "cssr_system"

#define SOFTWARE_VERSION "v1.0"

#define INITIALIZATION_INFO_PERIOD 5.0
#define OPERATION_INFO_PERIOD 10.0


// The test report file variables
extern std::string test_report_file ;                                         // data filename
extern std::string test_report_path;                                          // data path
extern std::string test_report_path_and_file;                                 // data path and filename

// The gesture execution test input file variables
extern bool run_iconic_gestures_test;                                          // Run iconic gestures test
extern bool run_deictic_gestures_test;                                         // Run deictic gestures test
extern bool run_bow_gestures_test;                                             // Run bow gestures test
extern bool run_nod_gestures_test;                                             // Run nod gestures test
extern bool run_symbolic_gestures_test;                                        // Run symbolic gestures test

extern std::string node_name;                                                 // The name of the ROS node

extern ros::Publisher speech_pub;                                            // Publisher for speech commands
extern std_msgs::String speech_msg;                                            // Message for speech commands


// The gesture execution unit test class
class GestureExecutionUnitTest : public ::testing::Test {
    protected:
        std::ofstream test_report;
        // Run before each test
        virtual void SetUp() {
            // Open the test report file in append mode
            test_report.open(test_report_path_and_file, std::ios::out | std::ios::app); 
            if (!test_report.is_open()) {
                ROS_ERROR("Failed to open test report file at %s", test_report_path_and_file.c_str());
            } else {
                ROS_INFO("Test report file opened successfully at %s", test_report_path_and_file.c_str());
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
        void logOtherTestResult(const std::string& testName, bool result) {
            if (test_report.is_open()) {
                test_report << testName;
                test_report << "\n\tResult        : " << (result ? "PASSED" : "FAILED") << "\n\n";
                test_report.flush();
            } else {
                ROS_ERROR("Test report file is not open.");
            }
        }

        // Log the iconic gesture test result to the test report file
        void logIconicTestResult(const std::string& testName, std::vector<std::vector<std::vector<std::string>>> gestureIdentifier, bool result) {
            if (test_report.is_open()) {
                test_report << testName;

                for (int i = 0; i < gestureIdentifier.size(); i++) {
                    for (int j = 0; j < gestureIdentifier[i].size(); j++) {
                        if(j % 2 == 0){
                            test_report << "\n\tJoint Names   :\t";
                            for (int k = 0; k < gestureIdentifier[i][j].size(); k++) {
                                test_report << gestureIdentifier[i][j][k] << " ";
                            }
                        }
                        else{
                            test_report << "\n\tJoint Angles  :\t";
                            for (int k = 0; k < gestureIdentifier[i][j].size(); k++) {
                                test_report << gestureIdentifier[i][j][k] << " ";
                                if (k < gestureIdentifier[i][j].size() - 1) {
                                    test_report << "\n\t\t\t\t\t";
                                }
                            }
                        }
                    }
                }        
                test_report << "\n\tResult        : " << (result ? "PASSED" : "FAILED") << "\n\n";
                test_report.flush();
            } else {
                ROS_ERROR("Test report file is not open.");
            }
        }
};


/*  
 *  Function to read the gesture execution test input file
 *
 *  The function reads the gesture execution test input file and sets the variables for the gesture execution tests
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 *      debug: the debug mode
 * 
 *  @return
 *      0 if the gesture execution test input file is read successfully
 *      1 if the gesture execution test input file is not read successfully
 */
int read_gesture_execution_test_configuration(string* platform, bool* debug);

/*  
*   Function to write a string to a file
 *
 *  The function writes a string to a file
 *  
 *  @param:
 *      filename: the name of the file to write to
 *      directory: the directory where the file is located in the project workspace
 *      content: the content to write to the file
 *      separator: the separator to use in the file
 * 
 *  @return
 *      0 if the file is written successfully
 *      1 if the file is not written successfully
 */
int write_string_to_file(string filename, std::string directory, std::vector<std::vector<std::string>> content, std::string separator);

/*  
 *  Function to delete a file
 *
 *  The function deletes a file
 *  
 *  @param:
 *      filename: the name of the file to delete
 *      directory: the directory where the file is located in the project workspace
 * 
 *  @return
 *      0 if the file is deleted successfully
 *      1 if the file is not deleted successfully
 */
int delete_file(string filename, std::string directory);

/*  
 *  Function to write the gesture execution configuration file
 *
 *  The function writes the gesture execution configuration file
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 *      interpolation: the interpolation type to use in the gesture execution tests
 *      gesture_descriptors: the gesture descriptors configuration file
 *      simulator_topics_filename: the simulator topics configuration file
 *      robot_topics_filename: the robot topics configuration file
 *      verbose_mode_input: the verbose mode input
 * 
 *  @return
 *      0 if the file is written successfully
 *      1 if the file is not written successfully
 */
int write_configuration_file(std::string platform, std::string interpolation, std::string gesture_descriptors, std::string simulator_topics_filename, std::string robot_topics_filename, std::string verbose_mode_input);

/*  Function to invoke the gesture execution service and return the response from the service
 *  
 * @param:
 *     cmd: the service command to run
 * 
 * @return
 *    result: the response from the service
 */
std::string invoke_service(const char* cmd);


/* 
 *   Function to verify if a topic is available
 * @param:
 *   topic_name: string to store the topic name
 * 
 * @return:
 *  boolean indicating if the topic is available
 */
bool is_topic_available(std::string topic_name);

/* 
 *   Function to verify if a service is available
 * @param:
 *   service_name: string to store the service name
 * 
 * @return:
 *  boolean indicating if the service is available
 */
bool is_service_available(const std::string& service_name);

/*  
 *   Function to handle the shutdown signal
 *   Cancels all active goals and shuts down ROS
 *  @param:
 *      sig: the signal
 *  @return:
 *      None
 */
void shut_down_handler(int sig);

#endif // GESTURE_EXECUTION_TEST_INTERFACE_H