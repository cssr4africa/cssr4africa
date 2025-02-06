/*
 * animateBehaviourTestInterface.h
 *
 * Author:  Eyerusalem Mamuye Birhan 
 * Email:   ebirhan@andrew.cmu.edu
 * Date:    2025-01-10
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
 

#ifndef ANIMATE_BEHAVIOUR_TEST_INTERFACE_H
#define ANIMATE_BEHAVIOUR_TEST_INTERFACE_H

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <sstream>
#include <ros/package.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <atomic>


// Global variables
#define CSSR_SYSTEM_PACKAGE_NAME "cssr_system"
#define UNIT_TEST_PACKAGE_NAME "unit_tests"

extern std::string nodeName; 
extern std::string testReportPathAndFile;
extern std::ofstream testReport;
static ros::Publisher speechPublisher; 


/**
 * @brief Controls heartbeat message showing node status
 * 
 * @param control "start" to begin heartbeat, "stop" to end it
 * Shows node running status every 10 seconds while active
 * Automatically cleans up resources when stopped
 */
void heartbeat(const std::string& control);

/**
 * @brief Performs cleanup operations for the animate behaviour node
 * 
 * This function ensures the animate behaviour node is properly shut down
 * and performs any necessary cleanup operations.
 */
void performCleanup();

/**
* @brief Signal handler for graceful program termination
* Ensures safe shutdown by disabling robot movements and restoring defaults
* @param signum Signal number triggering the shutdown
*/
void signalHandler(int sig);

/**
 * @brief Writes the initial configuration settings to a file.
 * 
 * This is used to set up animate behavior configuration file values that will 
 * modify the existing animate behavior node configuration values.
 */
void writeStringToFile(const std::string& filename, 
                         const std::string& directory, 
                         const std::vector<std::vector<std::string>>& content, 
                         const std::string& separator);

/**
 * @brief Generates and writes configuration settings to a file based on a map of configuration values.
 * 
 * This function takes a map containing various robot configuration parameters, such as platform type, 
 * behavior, range limits for movement, and timing information, and writes them into a structured format 
 * in a configuration file. Each parameter is separated by a specified separator for readability and consistency.
 * 
 * @param configValues A map containing the configuration parameters as key-value pairs.
 */
void writeConfigurationFile(const std::map<std::string, std::string>& configValues); 

/**
 * @brief Writes structured data to a specified file.
 * 
 * This function writes a two-dimensional vector of strings to a file located in a specified directory. 
 * Each inner vector represents a row in the file, and the elements within each row are separated by 
 * the provided separator. If the file already exists, its contents are overwritten.
 * 
 * @param filename Name of the file to write data to.
 * @param directory Directory path where the file will be saved.
 * @param content 2D vector of strings, with each inner vector representing a line in the file.
 * @param separator String used to separate elements within each row.
 */
void setConfigurationAndWriteFile(const std::string& behaviour);

/**
 * @brief Executes a shell command and retrieves the output.
 * 
 * This function runs a given shell command using the `popen` function, capturing its output
 * in real-time. The output is stored and returned as a string. If the command fails, 
 * an exception is thrown.
 * 
 * @param cmd The shell command to execute.
 * @return The output of the command as a string.
 * @throws std::runtime_error if the command execution fails.
 */
std::string invokeService(const char* cmd);

/**
 * @brief Reads test configuration settings from a file.
 * 
 * This function loads configuration settings from the specified file and stores them 
 * in a map with boolean values for each setting. It expects a key-value format, 
 * where the value is converted to a boolean based on the presence of 'true' or 'false'.
 * 
 * @param configFilePath Path to the configuration file.
 * @return A map with configuration keys and boolean values.
 */
std::map<std::string, bool> readTestConfig(const std::string& configFilePath);


/**
 * @brief Writes the initial configuration settings to a file.
 * 
 * This is used to set up animate behavior configuration file values that will 
 * modify the existing animate behavior node configuration values.
 */
void writeInitialConfigurationFile();

/**
 * @brief Changes the animate behavior state via ROS service call.
 * 
 * This function sends a service request to change the robot's animate behavior state 
 * to either "enabled" or "disabled." It returns true if the service call indicates success.
 * 
 * @param state The desired state for animate behavior ("enabled" or "disabled").
 * @return True if the service call was successful, false otherwise.
 */
bool setAnimateBehaviour(const std::string& state);

/**
 * @brief Reads configuration settings from a file.
 * 
 * This function reads all configuration settings from a specified file located in the 
 * animateBehaviour/config directory. It processes each line of the file and extracts
 * all configuration values, including the behaviour setting which specifies the type
 * of animation (hands, body, rotation, or all).
 * 
 * @return std::map<std::string, std::string> Map containing all configuration values
 */
std::map<std::string, std::string> readBehaviorConfig();

/**
 * @brief Reads a log file to verify and capture animate behavior details.
 * 
 * This function checks the log file for specific animate behavior messages, 
 * captures relevant values (e.g., configuration, joint details, positions), 
 * and returns them to the calling test function for validation.
 * 
 * @param logFilePath Path to the log file.
 * @param configContent, jointNames, homePositions, randomPositions Maps to store extracted data.
 * @return Captured values and state details to the test function.
 */
void processLogFile(const std::string& logFilePath, 
                      const std::string& enableMsg,
                      const std::string& disableMsg,
                      const std::string& startMovementMsg,
                      const std::string& endMovementMsg,
                      bool& behaviourEnabled,
                      bool& behaviourDisabled,
                      bool& movementStarted,
                      bool& movementEnded,
                      std::map<std::string, std::string>& configContent,
                      std::map<std::string, std::string>& jointNames,
                      std::map<std::string, std::vector<std::string>>& homePositions,
                      std::map<std::string, std::vector<std::string>>& randomPositions);


/**
 * @brief Restores all configuration settings to their default values after tests
 * 
 * This function is called after test completion to ensure the configuration file
 * is reset to its default state. This includes setting the behaviour value back
 * to 'body' and restoring all other parameters to their default values.
 */
void restoreDefaultConfiguration();

/**
 * @class AnimateBehaviourRobotTest
 * @brief Test suite for validating animate behavior of the robot.
 * 
 * Provides setup, teardown, and helper functions to maintain a clean state 
 * between tests. Manages test configurations, logging, and ensures the animate 
 * behavior is disabled before and after each test.
 * 
 * @details
 * - Uses Google Test framework for unit testing.
 * - Manages test report streams for logging test results.
 * - Provides utility methods for resetting the robot state and managing configurations.
 * - Ensures consistent behavior state throughout the test lifecycle.
 */
class AnimateBehaviourRobotTest : public ::testing::Test {
    protected:
        ros::Publisher speechPublisher;
        void announceSpeech(const std::string& testType);  

        static std::ofstream* testReport;
        static bool alert;

        void SetUp() override {
            ros::NodeHandle nh;
            speechPublisher = nh.advertise<std_msgs::String>("/speech", 1);
            // Remove the initial disable call and just wait for ROS initialization
            ros::Duration(2.0).sleep();
        }

        void TearDown() override {
            try {
                
                // Ensure test report is flushed
                if (testReport && testReport->is_open()) {
                    testReport->flush();
                }
            } catch (const std::exception& e) {
                ROS_WARN("Exception in TearDown: %s", e.what());
            }
        }

    public:
        static void setTestReportStream(std::ofstream& stream) {
            testReport = &stream;
            if (testReport && testReport->is_open()) {
                testReport->flush();
            }
        }

        static std::ofstream& getTestReportStream() {
            if (!testReport || !testReport->is_open()) {
                throw std::runtime_error("Test report stream is not initialized or not open");
            }
            testReport->flush();
            return *testReport;
        }
};

#endif // ANIMATE_BEHAVIOUR_TEST_INTERFACE_H