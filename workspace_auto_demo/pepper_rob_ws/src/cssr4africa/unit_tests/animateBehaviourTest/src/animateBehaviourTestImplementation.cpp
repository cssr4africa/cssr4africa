/* animateBehaviourTestImplementation.cpp
 *
 * Author:  Eyerusalem Mamuye Birhan
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

#include "animateBehaviourTest/animateBehaviourTestInterface.h"

std::ofstream* AnimateBehaviourRobotTest::testReport = nullptr;

bool AnimateBehaviourRobotTest::alert = false;                          // Initialize alert to false initially
static bool cleanupPerformed = false;
// Capture the node name
std::string nodeName;  



/**
 * @brief Controls heartbeat message showing node status
 * 
 * @param control "start" to begin heartbeat, "stop" to end it
 * Shows node running status every 10 seconds while active
 * Automatically cleans up resources when stopped
 */
void heartbeat(const std::string& control) {
    static boost::thread* heartbeatThread = nullptr;
    static std::atomic<bool> shouldRun{true};

    if (control == "start") {
        shouldRun = true;
        heartbeatThread = new boost::thread([&]() {
            while (shouldRun && ros::ok()) {
                ROS_INFO("%s: running.", nodeName.c_str());
                boost::this_thread::sleep_for(boost::chrono::seconds(10));
            }
        });
    } 
    else if (control == "stop") {
        if (heartbeatThread) {
            shouldRun = false;
            heartbeatThread->join();
            delete heartbeatThread;
            heartbeatThread = nullptr;
        }
    }
}

/**
* @brief Signal handler for graceful program termination
* Ensures safe shutdown by disabling robot movements and restoring defaults
* @param signum Signal number triggering the shutdown
*/
void signalHandler(int sig) {
    try {
        // Stop the heartbeat
        heartbeat("stop");
        ROS_ERROR("%s: Node has been killed/terminated unexpectedly", nodeName.c_str());
        
        // Force immediate shutdown
        ros::shutdown();
        _exit(0);
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] sigintHandler: Error during shutdown cleanup: %s", nodeName.c_str(), e.what());
        _exit(1);
    }
}


/**
 * @brief Performs cleanup operations for the animate behaviour node
 * 
 * This function ensures the animate behaviour node is properly shut down
 * and performs any necessary cleanup operations.
 */
void performCleanup() {
    if (!cleanupPerformed) {
        ROS_INFO("Performing cleanup operations...");
        setAnimateBehaviour("disabled");
        ros::Duration(1.0).sleep();
    }
}


// Helper function to trim whitespace from both ends of a string
std::string trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\n\r");
    size_t end = str.find_last_not_of(" \t\n\r");

    return (start == std::string::npos || end == std::string::npos) ? "" : str.substr(start, end - start + 1);
}

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
std::map<std::string, std::string> readBehaviorConfig() {
    std::map<std::string, std::string> configValues;
    
    std::string dataPath = ros::package::getPath(UNIT_TEST_PACKAGE_NAME);
    std::string configFilePath = dataPath + "/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini";
    
    std::ifstream configFile(configFilePath);
    if (configFile.is_open()) {
        std::string line;
        while (std::getline(configFile, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            std::string key, value;
            if (iss >> key) {
                std::getline(iss >> std::ws, value);
                key = trim(key);
                value = trim(value);

                // Store configuration value
                configValues[key] = value;
                // ROS_INFO("%s: Configuration value: %s = %s",  nodeName.c_str(),key.c_str(), value.c_str());
            }
        }
        configFile.close();
    } else {
        ROS_ERROR("%s: Failed to open config file at %s", nodeName.c_str(), configFilePath.c_str());
    }

    return configValues;
}

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
void writeStringToFile(const std::string& filename, const std::string& directory, const std::vector<std::vector<std::string>>& content, const std::string& separator) {
    std::string dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME).c_str();
    ROS_INFO(" %s: Base path from ros::package::getPath: %s", nodeName.c_str(), dataPath.c_str());
    dataPath += "/animateBehaviour";
    dataPath += directory;
    std::string dataPathAndFile = dataPath + filename;
    
    std::ofstream dataFile(dataPathAndFile, std::ios::out | std::ios::trunc); // Clear the file
    if (dataFile.is_open()) {
        for (int i = 0; i < content.size(); i++) {
            for (int j = 0; j < content[i].size(); j++) {
                dataFile << content[i][j] << separator;
            }
            dataFile << "\n";
        }
        dataFile.close();
    } else {
        ROS_ERROR("%s: Failed to open data file at %s", nodeName.c_str(), dataPathAndFile.c_str());
    }
}

/**
 * @brief Generates and writes configuration settings to a file based on a map of configuration values.
 * 
 * This function takes a map containing various robot configuration parameters, such as platform type, 
 * behavior, range limits for movement, and timing information, and writes them into a structured format 
 * in a configuration file. Each parameter is separated by a specified separator for readability and consistency.
 * 
 * @param configValues A map containing the configuration parameters as key-value pairs.
 */
void writeConfigurationFile(const std::map<std::string, std::string>& configValues) {
    std::string separator = "\t\t\t";
    std::string dataDirectory = "/config/";

    // Extract values from the map with defaults for missing keys
    std::string platform = configValues.count("platform") ? configValues.at("platform") : "default_platform";
    std::string behaviour = configValues.count("behaviour") ? configValues.at("behaviour") : "default_behaviour";
    std::string simulatorTopicsFilename = configValues.count("simulatorTopicsFilename") ? configValues.at("simulatorTopicsFilename") : "simulatorTopics.dat";
    std::string robotTopicsFilename = configValues.count("robotTopicsFilename") ? configValues.at("robotTopicsFilename") : "pepperTopics.dat";
    std::string verboseModeInput = configValues.count("verboseModeInput") ? configValues.at("verboseModeInput") : "false";
    std::string rotMaximumRange = configValues.count("rotMaximumRange") ? configValues.at("rotMaximumRange") : "0.3";
    std::string selectedRange = configValues.count("selectedRange") ? configValues.at("selectedRange") : "0.5";
    std::string armMaximumRange = configValues.count("armMaximumRange") ? configValues.at("armMaximumRange") : "0.2,0.2,0.2,0.35,0.2";
    std::string handMaximumRange = configValues.count("handMaximumRange") ? configValues.at("handMaximumRange") : "0.7";
    std::string legMaximumRange = configValues.count("legMaximumRange") ? configValues.at("legMaximumRange") : "0.1,0.1,0.08";
    std::string duration = configValues.count("gestureDuration") ? configValues.at("gestureDuration") : "1.0";
    std::string numPoints = configValues.count("numPoints") ? configValues.at("numPoints") : "100";
    std::string numPointsLeg = configValues.count("numPointsLeg") ? configValues.at("numPointsLeg") : "2";
    std::string legRepeatFactor = configValues.count("legRepeatFactor") ? configValues.at("legRepeatFactor") : "8";

    // Prepare the configuration content
    std::vector<std::vector<std::string>> configurationContent = {
        {"platform\t\t", platform},
        {"behaviour\t\t", behaviour},
        {"simulatorTopics\t", simulatorTopicsFilename},
        {"robotTopics\t\t", robotTopicsFilename},
        {"verboseMode\t\t", verboseModeInput},
        {"rotMaximumRange\t", rotMaximumRange},
        {"selectedRange\t", selectedRange},
        {"armMaximumRange\t", armMaximumRange},
        {"handMaximumRange\t", handMaximumRange},
        {"legMaximumRange\t", legMaximumRange},
        {"gestureDuration\t", duration},
        {"numPoints\t\t", numPoints},
        {"numPointsLeg\t", numPointsLeg},
        {"legRepeatFactor\t", legRepeatFactor}
    };

    // Write the configuration to the file
    writeStringToFile("animateBehaviourConfiguration.ini", dataDirectory, configurationContent, separator);
}

/**
 * @brief Sets the specified behavior in the configuration file and saves the updated file.
 * 
 * This function reads the existing configuration file, updates the behavior setting 
 * with the specified value, and then saves the modified configuration. This ensures 
 * that the behavior is correctly applied before initiating the test.
 * 
 * @param behaviour Desired behavior setting to apply (e.g., 'hands', 'body').
 */
void setConfigurationAndWriteFile(const std::string& behaviour, const std::string& platform) {
    // Get absolute path
    std::string configFilePath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME) + 
                                "/animateBehaviour/config/animateBehaviourConfiguration.ini";
    
    // Read and store all current config values
    std::map<std::string, std::string> currentConfig;
    std::ifstream inputFile(configFilePath);
    std::string line;

    while (std::getline(inputFile, line)) {
        size_t delimiterPos = line.find("\t\t");
        if (delimiterPos != std::string::npos) {
            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 2);
            currentConfig[key] = value;
        }
    }
    inputFile.close();

    // Update the configuration with new values
    currentConfig["behaviour"] = behaviour;
    currentConfig["platform"] = platform;

    // Write updated configuration back to file
    std::ofstream outputFile(configFilePath);
    for (const auto& configPair : currentConfig) {
        outputFile << configPair.first << "\t\t" << configPair.second << std::endl;
    }
    outputFile.close();

    ROS_INFO("%s: Updated configuration file with behaviour: %s and platform: %s", 
             nodeName.c_str(), behaviour.c_str(), platform.c_str());
}

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

/**
 * @brief Changes the animate behavior state via ROS service call.
 * 
 * This function sends a service request to change the robot's animate behavior state 
 * to either "enabled" or "disabled." It returns true if the service call indicates success.
 * 
 * @param state The desired state for animate behavior ("enabled" or "disabled").
 * @return True if the service call was successful, false otherwise.
 */
bool setAnimateBehaviour(const std::string& state) {
    std::string cmd = "rosservice call /animateBehaviour/set_activation \"state: '" + state + "'\"";
    std::string result = invokeService(cmd.c_str());
    if (result.find("success") != std::string::npos) {
        return true;
    } else {
        ROS_ERROR(" %s: Service call failed or unexpected response: %s", nodeName.c_str(), result.c_str());
        return false;
    }
}

/**
 * @brief Announces test status using Pepper's text-to-speech.
 * 
 * @param testType Type of test being run ("flexi", "body", "rotation", or "all")
 * @param isStarting True if test is starting, false if ending
 */
void AnimateBehaviourRobotTest::announceSpeech(const std::string& testType) {
    if (!ros::ok()) {
        ROS_ERROR("ROS not OK when trying to announce speech");
        return;
    }

    try {
        // Initialize the publisher if not already initialized
        if (!speechPublisher) {
            ros::NodeHandle nh;
            speechPublisher = nh.advertise<std_msgs::String>("/speech", 1);
            ROS_INFO("%s: Created speech publisher on topic /speech", nodeName.c_str());
            ros::Duration(1.0).sleep(); // Give more time for publisher to initialize
        }

        std_msgs::String msg;
        std::string announcement;

        // Set appropriate announcement message
        if (testType == "flexi") {
            announcement = "Now I am running test one, testing flexi hand movements.";
        }
        else if (testType == "body") {
            announcement = "Now I am running test two, testing body movements.";
        }
        else if (testType == "rotation") {
            announcement = "Now I am running test three, testing rotation movements.";
        }
        else if (testType == "all") {
            announcement = "Now I am running test four, testing all movements combined.";
        }

        msg.data = announcement;
        
        // Check if we have subscribers before publishing
        if (speechPublisher.getNumSubscribers() == 0) {
            ROS_WARN("%s: No subscribers to /speech topic. Waiting for subscribers...", nodeName.c_str());
            ros::Duration(2.0).sleep();  // Wait for subscribers
        }

        ROS_INFO("%s: Publishing speech message: %s", nodeName.c_str(), announcement.c_str());
        speechPublisher.publish(msg);
        
        // Wait for message to be processed
        ros::spinOnce();
        ros::Duration(3.0).sleep();  // Give more time for speech to complete

    } catch (const std::exception& e) {
        ROS_ERROR("%s: Exception in announceSpeech: %s", nodeName.c_str(), e.what());
    }
}

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
                      std::map<std::string, std::vector<std::string>>& randomPositions)  {
    
    // Clear any existing content in the maps
    configContent.clear();
    jointNames.clear();
    homePositions.clear();
    randomPositions.clear();

    std::string line;

    std::ifstream logFile(logFilePath);
    ASSERT_TRUE(logFile.is_open()) << "Failed to open log file at " << logFilePath;

    // First read the configuration file
    std::string configFilePath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME) + 
                                "/animateBehaviour/config/animateBehaviourConfiguration.ini";

    std::ifstream configFile(configFilePath);
    ASSERT_TRUE(configFile.is_open()) << "Failed to open config file at " << configFilePath;
    
    /* Read the configuration values used by the animate behvaiour node*/
    while (std::getline(configFile, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string key, value;
        if (iss >> key) {
            std::getline(iss >> std::ws, value);
            key = trim(key);
            value = trim(value);
            configContent[key] = value;
        }
    }
    configFile.close();

    bool insideMovement = false;
    bool readingRandomPositions = false;
    std::string currentJointType;
    int positionsToRead = 0;  // Number of positions to read

    while (std::getline(logFile, line)) {
        // Trim the line to avoid any leading/trailing whitespace issues
        line = trim(line);

        // Detect the start of the movement
        if (line.find(startMovementMsg) != std::string::npos) {
            insideMovement = true;
            movementStarted = true;
            //ROS_INFO("Found start movement message: %s", line.c_str());
            continue;
        }

        // Detect the end of the movement
        if (line.find(endMovementMsg) != std::string::npos) {
            insideMovement = false;
            readingRandomPositions = false;
            positionsToRead = 0;
            movementEnded = true;
            //ROS_INFO("Found end movement message: %s", line.c_str());
            continue;
        }

        // Handle enable/disable behavior status
        if (line.find(enableMsg) != std::string::npos) {
            behaviourEnabled = true;
            //ROS_INFO("Behaviour enabled message found.");
        }
        if (line.find(disableMsg) != std::string::npos) {
            behaviourDisabled = true;
           // ROS_INFO("Behaviour disabled message found.");
        }

        if (insideMovement) {
            // Capture joint topic and names
            std::string topicPrefix = "The topic name for the ";
            std::string jointNamesPrefix = "The joint names for ";
            std::string jointHomePrefix = "The home position of the ";
            std::string randomPositionsPrefix = "The random positions of the ";
            std::string generatedPositionsPrefix = "The size of the generated random positions are: ";

            if (line.find(topicPrefix) != std::string::npos) {
                size_t pos = line.find(" is ");
                if (pos != std::string::npos) {
                    currentJointType = line.substr(topicPrefix.length(), pos - topicPrefix.length());
                }
            }

            // Capture joint names
            if (line.find(jointNamesPrefix) != std::string::npos && line.find(" are: ") != std::string::npos) {
                size_t pos = line.find(" are: ");
                std::string names = line.substr(pos + 6); // after " are: "
                jointNames[currentJointType] = names;
            }

            // Capture home position
            if (line.find(jointHomePrefix) != std::string::npos && line.find(" is ") != std::string::npos) {
                size_t pos = line.find(" is ");
                std::string homePos = line.substr(pos + 4); // after " is "
                homePositions[currentJointType].push_back(homePos);
            }

            // Start reading random positions (after reading the number of positions)
            if (line.find(generatedPositionsPrefix) != std::string::npos) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    try {
                        positionsToRead = std::stoi(line.substr(pos + 1)); // Read the number of positions
                        readingRandomPositions = true;
                    } catch (std::exception& e) {
                        ROS_ERROR("Failed to convert number of positions to read: %s", e.what());
                    }
                }
                continue;
            }

            // Collect the random positions as full lines without parsing
            if (readingRandomPositions && positionsToRead > 0 && line.find("[") != std::string::npos) {
                // Capture the line as it is
                std::string rawPosition = line; // Store the full line
                
                // Remove any extra spaces (optional)
                rawPosition.erase(std::remove_if(rawPosition.begin(), rawPosition.end(), ::isspace), rawPosition.end());

                // Push the raw string directly to the randomPositions map for the current joint
                if (!rawPosition.empty()) {
                    // Store the raw position string as part of random positions
                    randomPositions[currentJointType].push_back({rawPosition}); // Treating it as a vector of strings
                    positionsToRead--;
                
                } else {
                    ROS_WARN("%s: Failed to capture a valid position from line: %s", nodeName.c_str(), line.c_str());
                }

                // Stop reading after the required number of positions
                if (positionsToRead == 0) {
                    readingRandomPositions = false;  // Stop reading when all positions are captured
                }
            }
        }
    }

    logFile.close();  // Close the log file

    //Clear the log file after processing
    std::ofstream logFileClear(logFilePath, std::ofstream::out | std::ofstream::trunc);
    ASSERT_TRUE(logFileClear.is_open()) << "Failed to open log file for clearing at " << logFilePath;
    logFileClear.close();
}

/* @brief Test case for Flexi Hand Animate Behavior functionality.
* 
* Validates the animation behavior by: checking config settings,
* enabling animation service, executing movement sequence, and
* verifying joint positions through log file analysis.
*/
TEST_F(AnimateBehaviourRobotTest, Test01FlexiHandAnimateBehaviour) {
        
        // Initial setup and delay
        ros::Duration(5.0).sleep();  

        // Get configuration values
        auto configValues = readBehaviorConfig();
        std::string currentBehavior = configValues["behaviour"];

        if (configValues["behaviour"] != "hands") {
        ROS_INFO("%s: Skipping Flexi Hand test - current behavior is: %s", nodeName.c_str(), configValues["behaviour"].c_str());
        GTEST_SKIP();
        }
        ROS_INFO("%s: Configuration parameters:\n"
         "                                                       platform: %s\n"
         "                                                       behaviour: %s\n"
         "                                                       simulatorTopics: %s\n"
         "                                                       robotTopics: %s\n"
         "                                                       verboseMode: %s\n"
         "                                                       rotMaximumRange: %s\n"
         "                                                       selectedRange: %s\n"
         "                                                       handMaximumRange: %s\n"
         "                                                       duration: %s\n"
         "                                                       numPoints: %s\n"
         "                                                       numPointsLeg: %s\n"
         "                                                       legRepeatFactor: %s\n"
         "                                                       armMaximumRange: %s\n"
         "                                                       legMaximumRange: %s",
         nodeName.c_str(),
         configValues["platform"].c_str(),
         configValues["behaviour"].c_str(),
         configValues["simulatorTopics"].c_str(),
         configValues["robotTopics"].c_str(),
         configValues["verboseMode"].c_str(),
         configValues["rotMaximumRange"].c_str(),
         configValues["selectedRange"].c_str(),
         configValues["handMaximumRange"].c_str(),
         configValues["duration"].c_str(),
         configValues["numPoints"].c_str(),
         configValues["numPointsLeg"].c_str(),
         configValues["legRepeatFactor"].c_str(),
         configValues["armMaximumRange"].c_str(),
         configValues["legMaximumRange"].c_str());
        
        announceSpeech("flexi");

        // Wait for service
        ROS_INFO("%s: Waiting for animate behaviour service...", nodeName.c_str());
        bool serviceAvailable = ros::service::waitForService("/animateBehaviour/set_activation", ros::Duration(10.0));
        if (!serviceAvailable) {
            ROS_ERROR("%s: Service /animateBehaviour/set_activation is not available after waiting", nodeName.c_str());
            FAIL() << "Required service /animateBehaviour/set_activation is not available.";
            return;
        }

        // Update configuration for hands test
        configValues["behaviour"] = "hands";
        writeConfigurationFile(configValues);
        ros::Duration(2.0).sleep(); 

        // Enable behavior and wait for initialization
        bool enableSuccess = setAnimateBehaviour("enabled");
        ASSERT_TRUE(enableSuccess) << "Failed to enable animate behaviour";
        ros::Duration(1.0).sleep();

        // Allow behavior to execute
        ros::Duration(60.0).sleep();

        // Disable behavior
        bool disableSuccess = setAnimateBehaviour("disabled");
        ASSERT_TRUE(disableSuccess) << "Failed to disable animate behaviour";
        ros::Duration(1.0).sleep();

        // Process log file and continue with test validation...
        std::string dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME);
        std::string logFilePath = dataPath + "/animateBehaviour/data/animateBehaviourLogFile.log";
        
        bool animateBehaviourEnabled = false;
        bool animateBehaviourDisabled = false;
        bool flexiMovementStarted = false;
        bool endFlexiMovement = false;
        std::map<std::string, std::string> configContent;
        std::map<std::string, std::string> jointNames;
        std::map<std::string, std::vector<std::string>> homePositions;
        std::map<std::string, std::vector<std::string>> targetPositions;


        processLogFile(logFilePath, 
                    "animateBehaviour node is active. Ready to animate.",
                    "Animate behaviour disabled.",
                    "[START FLEXI MOVEMENT]",  
                    "[END FLEXI MOVEMENT]",    
                    animateBehaviourEnabled,
                    animateBehaviourDisabled,
                    flexiMovementStarted,
                    endFlexiMovement,
                    configContent,
                    jointNames,
                    homePositions,
                    targetPositions); // Correctly initialize and pass

        // Log the test result

        
        auto& testReport = getTestReportStream();
        ASSERT_TRUE(testReport.is_open()) << "Failed to open test report file.";
        testReport << "===============================================================================================\n";
        testReport << "Animate behaviour enabled: " << (animateBehaviourEnabled ? "PASSED" : "FAILED") << "\n";
        testReport << "...............................................................................................\n";
        testReport << "Test 1: Test Flexi Hand Animate Behavior\n";
        testReport << "\n";
    
        testReport << "        Configuration Settings:\n";
        for (const auto& pair : configContent) {
            testReport << "        " << std::setw(20) << std::left << pair.first << ": " << pair.second << "\n";
        }
        testReport << "\n";

        testReport << "        Flexi movement started: " << (flexiMovementStarted ? "PASSED" : "FAILED") << "\n";
        testReport << "\n";

        testReport << "        Joint names:\n";
        for (const auto& pair : jointNames) {
            testReport << "            " << pair.first << ": " << pair.second << "\n";
        }
        testReport << "\n";

        testReport << "        Ensure the joint moves to the Home position before starting random movements.\n";
        testReport << "        The values of the home positions are:\n";
        for (const auto& pair : homePositions) {
            testReport << "            " << pair.first << ": ";
            for (const auto& pos : pair.second) {
                testReport << pos << " ";
            }
            testReport << "\n";

        }
        testReport << "\n";
        testReport << "        After the joint is in the home position, start moving to random positions continuously.\n";
        testReport << "        The random positions captured are:\n";
        for (const auto& pair : targetPositions) {
            testReport << "            " << pair.first << ": ";
            testReport << "\n";
            for (const auto& pos : pair.second) {
                testReport << "            "  << pos << " ";
                testReport << "\n";
            }
            testReport << "\n";
        }

        
        testReport << "        Flexi movement ended: " << (endFlexiMovement ? "PASSED" : "FAILED") << "\n";
        testReport << "...............................................................................................\n";
        testReport << "\n";
        testReport << "Animate behaviour disabled: " << (animateBehaviourDisabled ? "PASSED" : "FAILED") << "\n";
        testReport << "===============================================================================================\n";

        // Assertions
        EXPECT_TRUE(animateBehaviourEnabled);
        EXPECT_TRUE(flexiMovementStarted);
        EXPECT_TRUE(endFlexiMovement);
        EXPECT_TRUE(animateBehaviourDisabled);
        EXPECT_FALSE(configContent.empty());
        EXPECT_FALSE(jointNames.empty());
        EXPECT_FALSE(homePositions.empty());
        EXPECT_FALSE(targetPositions.empty());

    } 
 
 /**
 * @brief Test case for the Subtle Body animate behavior
 * 
 * This test verifies the functionality of the body animation behavior.
 * It only executes if the behavior setting in the configuration file is "body".
 * The test validates the body movement patterns and their execution sequence.
 */
TEST_F(AnimateBehaviourRobotTest, Test02SubtleBodyAnimateBehaviour) {
        //Get configuration values
        auto configValues = readBehaviorConfig();
        
        // Skip if behavior is not "body"
        if (configValues["behaviour"] != "body") {
            ROS_INFO("%s: Skipping Subtle Body test - current behavior is: %s", nodeName.c_str(), configValues["behaviour"].c_str());
            GTEST_SKIP();
        }
        ROS_INFO("%s: Configuration parameters:\n"
         "                                                       platform: %s\n"
         "                                                       behaviour: %s\n"
         "                                                       simulatorTopics: %s\n"
         "                                                       robotTopics: %s\n"
         "                                                       verboseMode: %s\n"
         "                                                       rotMaximumRange: %s\n"
         "                                                       selectedRange: %s\n"
         "                                                       handMaximumRange: %s\n"
         "                                                       duration: %s\n"
         "                                                       numPoints: %s\n"
         "                                                       numPointsLeg: %s\n"
         "                                                       legRepeatFactor: %s\n"
         "                                                       armMaximumRange: %s\n"
         "                                                       legMaximumRange: %s",
         nodeName.c_str(),
         configValues["platform"].c_str(),
         configValues["behaviour"].c_str(),
         configValues["simulatorTopics"].c_str(),
         configValues["robotTopics"].c_str(),
         configValues["verboseMode"].c_str(),
         configValues["rotMaximumRange"].c_str(),
         configValues["selectedRange"].c_str(),
         configValues["handMaximumRange"].c_str(),
         configValues["duration"].c_str(),
         configValues["numPoints"].c_str(),
         configValues["numPointsLeg"].c_str(),
         configValues["legRepeatFactor"].c_str(),
         configValues["armMaximumRange"].c_str(),
         configValues["legMaximumRange"].c_str());
        
        announceSpeech("body");

        bool serviceAvailable = ros::service::waitForService("/animateBehaviour/set_activation", ros::Duration(10.0));
        if (!serviceAvailable) {
            ROS_ERROR("%s: Service /animateBehaviour/set_activation is not available after waiting", nodeName.c_str());
            FAIL() << "Required service /animateBehaviour/set_activation is not available.";
            return;
        }

        // Update configuration for hands test
        configValues["behaviour"] = "body";
        writeConfigurationFile(configValues);
        ros::Duration(2.0).sleep(); 

        // Enable behavior and wait for initialization
        bool enableSuccess = setAnimateBehaviour("enabled");
        ASSERT_TRUE(enableSuccess) << "Failed to enable animate behaviour";
        ros::Duration(1.0).sleep();

        // Allow behavior to execute
        ros::Duration(50.0).sleep();

        // Disable behavior
        bool disableSuccess = setAnimateBehaviour("disabled");
        ASSERT_TRUE(disableSuccess) << "Failed to disable animate behaviour";
        ros::Duration(1.0).sleep();

    // Open the log report file, which is found in the unit test data folder
    std::string dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME);
    std::string logFilePath = dataPath + "/animateBehaviour/data/animateBehaviourLogFile.log";
    
    bool animateBehaviourEnabled = false;
    bool animateBehaviourDisabled = false;
    bool subtleBodyMovementStarted = false;
    bool endSubtleBodyMovement = false;
    std::map<std::string, std::string> configContent;
    std::map<std::string, std::string> jointNames;
    std::map<std::string, std::vector<std::string>> homePositions;
   std::map<std::string, std::vector<std::string>> targetPositions; 
    processLogFile(logFilePath, 
                     "animateBehaviour node is active. Ready to animate.",
                     "Animate behaviour disabled.",
                     "[START SUBTLE BODY MOVEMENT]",
                     "[END SUBTLE BODY MOVEMENT]",
                     animateBehaviourEnabled,
                     animateBehaviourDisabled,
                     subtleBodyMovementStarted,
                     endSubtleBodyMovement,
                     configContent,
                     jointNames,
                     homePositions,
                     targetPositions);  // Pass the correctly typed map

    // Log the test result
    auto& testReport = getTestReportStream();
    ASSERT_TRUE(testReport.is_open()) << "Failed to open test report file.";
    testReport << "\n";
    testReport << "\n";
    testReport << "===============================================================================================\n";
    testReport << "Animate behaviour enabled: " << (animateBehaviourEnabled ? "PASSED" : "FAILED") << "\n";
    testReport << "...............................................................................................\n";
    testReport << "Test 2: Test Subtle Body Animate Behavior\n";
    
    testReport << "        Configuration Settings:\n";
        for (const auto& pair : configContent) {
            testReport << "        " << std::setw(20) << std::left << pair.first << ": " << pair.second << "\n";
        }
        testReport << "\n";
    testReport << "        Subtle body movement started: " << (subtleBodyMovementStarted ? "PASSED" : "FAILED") << "\n";
    testReport << "\n";
    testReport << "        The joints used for the subtle body movements are:\n";
    for (const auto& pair : jointNames) {
        testReport << "            " << pair.first << ": " << pair.second << "\n";
    }
    testReport << "\n";
    testReport << "        Ensure the joint moves to the Home position before starting random movements.\n";
    testReport << "        The values of the home positions are:\n";
    for (const auto& pair : homePositions) {
        testReport << "            " << pair.first << ": ";
        for (const auto& pos : pair.second) {
            testReport << pos << " ";
        }
        testReport << "\n";
    }
    testReport << "\n";
    testReport << "        After the joint is in the home position, start moving to random positions continuously.\n";
    testReport << "        The random positions captured are:\n";
    for (const auto& pair : targetPositions) {
        testReport << "            " << pair.first << ": ";
        testReport << "\n";
        for (const auto& pos : pair.second) {
            testReport << "            "  << pos << " ";
            testReport << "\n";
        }
        testReport << "\n";
    }

    testReport << "        Subtle body movement ended: " << (endSubtleBodyMovement ? "PASSED" : "FAILED") << "\n";
    testReport << "...............................................................................................\n";
    testReport << "Animate behaviour disabled: " << (animateBehaviourDisabled ? "PASSED" : "FAILED") << "\n";
    testReport << "===============================================================================================\n";

    // Assertions
    EXPECT_TRUE(animateBehaviourEnabled);
    EXPECT_TRUE(subtleBodyMovementStarted);
    EXPECT_TRUE(endSubtleBodyMovement);
    EXPECT_TRUE(animateBehaviourDisabled);
    EXPECT_FALSE(configContent.empty());
    EXPECT_FALSE(jointNames.empty());
    EXPECT_FALSE(homePositions.empty());
    EXPECT_FALSE(targetPositions.empty());
}

TEST_F(AnimateBehaviourRobotTest, Test03RotationAnimateBehaviour) {
     // Get configuration values
        auto configValues = readBehaviorConfig();
        
        // Skip if behavior is not "rotation"
        if (configValues["behaviour"] != "rotation") {
            ROS_INFO("%s: Skipping Rotation test - current behavior is: %s", nodeName.c_str(), configValues["behaviour"].c_str());
           GTEST_SKIP();

        }
        
        announceSpeech("rotation");

        bool serviceAvailable = ros::service::waitForService("/animateBehaviour/set_activation", ros::Duration(10.0));
        if (!serviceAvailable) {
            ROS_ERROR("%s: Service /animateBehaviour/set_activation is not available after waiting", nodeName.c_str());
            FAIL() << "Required service /animateBehaviour/set_activation is not available.";
            return;
        }

        // Update configuration for hands test
        configValues["behaviour"] = "rotation";
        writeConfigurationFile(configValues);
        ros::Duration(2.0).sleep(); 

        // Enable behavior and wait for initialization
        bool enableSuccess = setAnimateBehaviour("enabled");
        ASSERT_TRUE(enableSuccess) << "Failed to enable animate behaviour";
        ros::Duration(1.0).sleep();

        // Allow behavior to execute
        ros::Duration(50.0).sleep();

        // Disable behavior
        bool disableSuccess = setAnimateBehaviour("disabled");
        ASSERT_TRUE(disableSuccess) << "Failed to disable animate behaviour";
        ros::Duration(1.0).sleep();

    // Open the log report file found in the unit test data folder
    std::string dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME);
    std::string logFilePath = dataPath + "/animateBehaviour/data/animateBehaviourLogFile.log";
    
    // Variables to store parsed log data
    bool animateBehaviourEnabled = false;
    bool animateBehaviourDisabled = false;
    bool rotationMovementStarted = false;
    bool endRotationMovement = false;
    std::map<std::string, std::string> configContent;
    std::map<std::string, std::string> jointNames;
    std::map<std::string, std::vector<std::string>> homePositions;  
    std::vector<double> randomPositions; // For storing random positions
    
    // Open log file and start processing
    std::ifstream logFile(logFilePath);
    ASSERT_TRUE(logFile.is_open()) << "Failed to open log file at " << logFilePath;

    std::string line;
    bool insideRandomPositions = false;  // To track if we are reading random positions
    int positionsToRead = 0;

    while (std::getline(logFile, line)) {
        // Trim the line to avoid leading/trailing whitespace issues
        line = trim(line);

        // Detect enabling of animate behaviour
        if (line.find("animateBehaviour node is active. Ready to animate.") != std::string::npos) {
            animateBehaviourEnabled = true;
        }

        // Detect disabling of animate behaviour
        if (line.find("Animate behaviour disabled.") != std::string::npos) {
            animateBehaviourDisabled = true;
        }

        // Parse config settings
        if (line.find("Config:") != std::string::npos) {
            size_t pos = line.find(" = ");
            if (pos != std::string::npos) {
                std::string key = line.substr(7, pos - 7);  // "Config:" is 7 chars long
                std::string value = line.substr(pos + 3);
                configContent[key] = value;
            }
        }

        // Detect start of rotation movement
        if (line.find("[START ROTATION BASE SHIFT]") != std::string::npos) {
            rotationMovementStarted = true;
        }

        // Detect end of rotation movement
        if (line.find("[END ROTATION BASE SHIFT]") != std::string::npos) {
            endRotationMovement = true;
        }

        // Parse joint names
        if (line.find("The joint names for the base are:") != std::string::npos) {
            size_t pos = line.find("[");
            if (pos != std::string::npos) {
                std::string jointName = line.substr(pos);
                jointNames["base"] = jointName;
            }
        }

        // Parse home position
        if (line.find("The home position of the base is") != std::string::npos) {
            size_t pos = line.find("is ");
            if (pos != std::string::npos) {
                std::string homePos = line.substr(pos + 3);
                homePositions["base"].push_back(homePos);
            }
        }

        // Detect the beginning of random positions
        if (line.find("The size of the generated random positions are:") != std::string::npos) {
            size_t pos = line.find(":");
            if (pos != std::string::npos) {
                positionsToRead = std::stoi(line.substr(pos + 2));
                insideRandomPositions = true;
                continue;
            }
        }

        // Read and store random positions
        if (insideRandomPositions && positionsToRead > 0) {
            try {
                double randomValue = std::stod(line);
                randomPositions.push_back(randomValue);
                positionsToRead--;
            } catch (const std::invalid_argument&) {
                // Handle error in case the line is not a number
            }

            // Stop reading random positions once all have been read
            if (positionsToRead == 0) {
                insideRandomPositions = false;
            }
        }
    }

    logFile.close();

    // Log the test result
    auto& testReport = getTestReportStream();
    ASSERT_TRUE(testReport.is_open()) << "Failed to open test report file.";
    testReport << "\n";
    testReport << "\n";
    testReport << "===============================================================================================\n";
    testReport << "Animate behaviour enabled: " << (animateBehaviourEnabled ? "PASSED" : "FAILED") << "\n";
    testReport << "...............................................................................................\n";
    testReport << "Test 3: Test Rotation in Z-axis Animate Behavior\n";
    testReport << "        Configuration Settings:\n";
        for (const auto& pair : configContent) {
            testReport << "        " << std::setw(20) << std::left << pair.first << ": " << pair.second << "\n";
        }
        testReport << "\n";
    testReport << "        Rotation in the z-axis started: " << (rotationMovementStarted ? "PASSED" : "FAILED") << "\n";
    testReport << "\n";
    testReport << "        The joints used for the rotation in the z-axis are:\n";
    for (const auto& pair : jointNames) {
        testReport << "            " << pair.first << ": " << pair.second << "\n";
    }
    testReport << "\n";
    testReport << "        Ensure the joint moves to the Home position before starting random movements.\n";
    testReport << "        The values of the home positions are:\n";
    for (const auto& pair : homePositions) {  
        testReport << "            " << pair.first << ": ";
        for (const auto& pos : pair.second) {
            testReport << pos << " ";
        }
        testReport << "\n";
    }
    testReport << "\n";
    testReport << "        After the joint is in the home position, start moving to random positions continuously.\n";
    testReport << "        The random position values generated are:\n";
    for (const auto& pos : randomPositions) {
        testReport << "            "  << pos << "\n";
    }
    
    testReport << "\n";
    testReport << "        Rotation in the z-axis ended: " << (endRotationMovement ? "PASSED" : "FAILED") << "\n";
    testReport << "...............................................................................................\n";
    testReport << "Animate behaviour disabled: " << (animateBehaviourDisabled ? "PASSED" : "FAILED") << "\n";

    // Assertions
    EXPECT_TRUE(animateBehaviourEnabled);
    EXPECT_TRUE(rotationMovementStarted);
    EXPECT_TRUE(endRotationMovement);
    EXPECT_TRUE(animateBehaviourDisabled);
    EXPECT_FALSE(configContent.empty());
    EXPECT_FALSE(jointNames.empty());
    EXPECT_FALSE(homePositions.empty());
    EXPECT_FALSE(randomPositions.empty());

}


/**
 * @brief Test case for all combined animate behaviors
 * 
 * This test verifies the functionality when all behaviors are running together.
 * It only executes if the behavior setting in the configuration file is "all".
 * The test validates that all movement types (hands, body, rotation) work correctly
 * when executed simultaneously.
 */
TEST_F(AnimateBehaviourRobotTest, Test04AllAnimateBehaviour) {
        // Get configuration values
        auto configValues = readBehaviorConfig();
        
        // Skip if behavior is not "rotation"
        if (configValues["behaviour"] != "All") {
            ROS_INFO("%s: Skipping all test - current behavior is: %s", nodeName.c_str(), configValues["behaviour"].c_str());
            GTEST_SKIP();
        }
        
        announceSpeech("all");

        bool serviceAvailable = ros::service::waitForService("/animateBehaviour/set_activation", ros::Duration(10.0));
        if (!serviceAvailable) {
            ROS_ERROR("%s: Service /animateBehaviour/set_activation is not available after waiting", nodeName.c_str());
            FAIL() << "Required service /animateBehaviour/set_activation is not available.";
            return;
        }

        // Update configuration for hands test
        configValues["behaviour"] = "All";
        writeConfigurationFile(configValues);
        ros::Duration(2.0).sleep(); 
        
        // Enable animate behaviour
        setAnimateBehaviour("enabled");
        
        // Wait for log messages to be generated
        ros::Duration(60.0).sleep();
        
        // Disable animate behaviour after the test
        setAnimateBehaviour("disabled");

        // Process log file
        std::string dataPath = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME);
        std::string logFilePath = dataPath + "/animateBehaviour/data/animateBehaviourLogFile.log";
        
        bool animateBehaviourEnabled = false;
        bool animateBehaviourDisabled = false;
        bool flexiMovementStarted = false;
        bool subtleBodyMovementStarted = false;
        bool rotationMovementStarted = false;
        bool flexiMovementEnded = false;
        bool subtleBodyMovementEnded = false;
        bool rotationMovementEnded = false;

        // Read and process log file
        std::ifstream logFile(logFilePath);
        ASSERT_TRUE(logFile.is_open()) << "Failed to open log file at " << logFilePath;

        std::string line;
        while (std::getline(logFile, line)) {
            if (line.find("animateBehaviour node is active. Ready to animate.") != std::string::npos) {
                animateBehaviourEnabled = true;
            }
            if (line.find("Animate behaviour disabled.") != std::string::npos) {
                animateBehaviourDisabled = true;
            }
            if (line.find("[START FLEXI MOVEMENT]") != std::string::npos) {
                flexiMovementStarted = true;
            }
            if (line.find("[START SUBTLE BODY MOVEMENT]") != std::string::npos) {
                subtleBodyMovementStarted = true;
            }
            if (line.find("[START ROTATION BASE SHIFT]") != std::string::npos) {
                rotationMovementStarted = true;
            }
            if (line.find("[END FLEXI MOVEMENT]") != std::string::npos) {
                flexiMovementEnded = true;
            }
            if (line.find("[END SUBTLE BODY MOVEMENT]") != std::string::npos) {
                subtleBodyMovementEnded = true;
            }
            if (line.find("[END ROTATION BASE SHIFT]") != std::string::npos) {
                rotationMovementEnded = true;
            }
        }
        logFile.close();

        // Calculate overall status
        bool allMovementsStarted = flexiMovementStarted && subtleBodyMovementStarted && rotationMovementStarted;
        bool allMovementsEnded = flexiMovementEnded && subtleBodyMovementEnded && rotationMovementEnded;

        // Log the test result
        auto& testReport = getTestReportStream();
        ASSERT_TRUE(testReport.is_open()) << "Failed to open test report file.";
        // Write test results to report
        testReport << "===============================================================================================\n";
        testReport << "Test 4: Test All Animate Behavior\n";
        testReport << "-----------------------------------------------------------------------------------------------\n";
        testReport << "Initialization:\n";
        testReport << "  Animate behaviour enabled: " << (animateBehaviourEnabled ? "PASSED" : "FAILED") << "\n\n";
        
        testReport << "Individual Movement Status:\n";
        testReport << "  Flexi Hand Movement:\n";
        testReport << "    Started: " << (flexiMovementStarted ? "PASSED" : "FAILED") << "\n";
        testReport << "    Ended:   " << (flexiMovementEnded ? "PASSED" : "FAILED") << "\n\n";
        
        testReport << "  Subtle Body Movement:\n";
        testReport << "    Started: " << (subtleBodyMovementStarted ? "PASSED" : "FAILED") << "\n";
        testReport << "    Ended:   " << (subtleBodyMovementEnded ? "PASSED" : "FAILED") << "\n\n";
        
        testReport << "  Rotation Movement:\n";
        testReport << "    Started: " << (rotationMovementStarted ? "PASSED" : "FAILED") << "\n";
        testReport << "    Ended:   " << (rotationMovementEnded ? "PASSED" : "FAILED") << "\n\n";
        
        testReport << "Overall Status:\n";
        testReport << "  All movements started: " << (allMovementsStarted ? "PASSED" : "FAILED") << "\n";
        testReport << "  All movements ended:   " << (allMovementsEnded ? "PASSED" : "FAILED") << "\n";
        testReport << "  Behaviour disabled:    " << (animateBehaviourDisabled ? "PASSED" : "FAILED") << "\n";
        testReport << "-----------------------------------------------------------------------------------------------\n";
        testReport << "Final Result: " << 
            ((animateBehaviourEnabled && allMovementsStarted && 
              allMovementsEnded && animateBehaviourDisabled) ? "PASSED" : "FAILED") << "\n";
        testReport << "===============================================================================================\n\n";

        // Ensure content is written
        testReport.flush();

        // Assertions for test validation
        EXPECT_TRUE(animateBehaviourEnabled) << "Failed to enable animate behaviour";
        EXPECT_TRUE(allMovementsStarted) << "Not all movements started";
        EXPECT_TRUE(allMovementsEnded) << "Not all movements ended";
        EXPECT_TRUE(animateBehaviourDisabled) << "Failed to disable animate behaviour";

}

/**
 * @brief Restores all configuration settings to their default values after tests
 * 
 * This function is called after test completion to ensure the configuration file
 * is reset to its default state. This includes setting the behaviour value back
 * to 'body' and restoring all other parameters to their default values.
 */
void restoreDefaultConfiguration() {
    std::map<std::string, std::string> defaultConfig;
    
    // Set all default values
    defaultConfig["platform"] = "robot";
    defaultConfig["behaviour"] = "body";
    defaultConfig["simulatorTopics"] = "simulatorTopics.dat";
    defaultConfig["robotTopics"] = "pepperTopics.dat";
    defaultConfig["verboseMode"] = "false";
    defaultConfig["rotMaximumRange"] = "0.3";
    defaultConfig["selectedRange"] = "0.5";
    defaultConfig["armMaximumRange"] = "0.15,0.15,0.2,0.35,0.2";
    defaultConfig["handMaximumRange"] = "0.7";
    defaultConfig["legMaximumRange"] = "0.1,0.1,0.08";
    defaultConfig["gestureDuration"] = "1.0";
    defaultConfig["numPoints"] = "100";
    defaultConfig["numPointsLeg"] = "2";
    defaultConfig["legRepeatFactor"] = "8";

    // Write the default configuration to file
    writeConfigurationFile(defaultConfig);
    ROS_INFO("%s: Configuration restored to default values", nodeName.c_str());
}

