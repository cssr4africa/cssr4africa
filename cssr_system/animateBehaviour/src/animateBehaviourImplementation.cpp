/* animateBehaviourImplementation.cpp
*
* Author: Eyerusalem Mamuye Birhan
* Date:   2025-01-10
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*/

#include "animateBehaviour/animateBehaviourInterface.h"

/* Global variables */
std::ofstream logFile;                                      // Runtime logs (auto-deleted after 10 minutes)
bool isActive = false;                                      // Controls the main behavior execution loop
bool verboseMode;                                           // Used to display diagnostic information
std::map<std::string, std::string> configParams;            // Configuration parameters
std::map<std::string, std::string> topicData;               // Platform-specific topic data
double maximumRange;                                        // Maximum movement range
double selectedRange;                                       // Selected movement range
const std::string FLAG_FILE_PATH = "first_run_flag.txt";    // Flag file path
boost::thread* heartbeatThread = nullptr;                   // Heartbeat thread
std::atomic<bool> shouldRunHeartbeat{true};                 // Heartbeat flag
std::string nodeName;                                        // Define the storage for nodeName



/* Joint name definitions for different robot parts */
std::vector<std::string> rArmJointNames = {
    "RShoulderPitch", 
    "RShoulderRoll",  
    "RElbowRoll", 
    "RElbowYaw", 
    "RWristYaw"
};

std::vector<std::string> lArmJointNames = {
    "LShoulderPitch",
    "LShoulderRoll", 
    "LElbowRoll", 
    "LElbowYaw", 
    "LWristYaw"};

std::vector<std::string> rhandJointNames = {"RHand"};
std::vector<std::string> lhandJointNames = {"LHand"};
std::vector<std::string> legJointNames = {
    "HipPitch", 
    "HipRoll", 
    "KneePitch"
};

/**
 * @brief Logs a message to the log file
 * @param message The message to log
 * 
 */
void logToFile(const std::string &message) {
    if (logFile.is_open()) {
        // Get the current time
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);

        // Write the timestamp and message to the log file
        logFile << std::ctime(&now_c) << message << std::endl;
        logFile.flush(); // Immediate flush to ensure data is written
    }
}

/**
 * Closes the log file safely
 */
void closeLogFile() {
    if (logFile.is_open()) {
        logFile.close();
    }
}

/**
 * @brief Main heartbeat function that runs in separate thread
 * 
 * Continuously outputs status message every 5 seconds to show node status.
 * Runs until shouldRunHeartbeat is false or ROS shuts down.
 */
void runHeartbeat() {
    while (shouldRunHeartbeat && ros::ok()) { 
        ROS_INFO("%s: running.", nodeName.c_str());
        boost::this_thread::sleep_for(boost::chrono::seconds(10));
    }
}

/**
 * @brief Initializes and starts the heartbeat thread
 * 
 * Creates new thread for status reporting.
 * Sets control flag to true and starts periodic messaging.
 */
void startHeartbeat() {
    shouldRunHeartbeat = true;
    heartbeatThread = new boost::thread(runHeartbeat);
}

/**
 * @brief Safely stops the heartbeat thread
 * 
 * Performs clean shutdown of heartbeat system:
 * 1. Checks if thread exists
 * 2. Signals thread to stop
 * 3. Waits for completion
 * 4. Frees memory
 * 5. Resets pointer
 */
void stopHeartbeat() {
    if (heartbeatThread) {
        shouldRunHeartbeat = false;
        heartbeatThread->join();
        delete heartbeatThread;
        heartbeatThread = nullptr;
    }
}


/**
 * Resets the animate behaviour by removing the flag file
 */
void resetAnimateBehaviour() {
     /* Remove the flag file to reset first run state */
    if (remove(FLAG_FILE_PATH.c_str()) != 0) {
        std::cerr << "logToFile: Error deleting file" << std::endl;
    }
}

/* Add after the global variables, before other functions */

/**
 * @brief Cancels all active movement goals and stops the robot
 * @param nh ROS NodeHandle for communication
 */
void cleanupAndCancelGoals(ros::NodeHandle& nh) {
    // ROS_INFO("Starting cleanup and goal cancellation...");
    try {
        // Only stop rotation movement if behavior is "All" or "rotation"
        if ((configParams["behaviour"] == "All" || configParams["behaviour"] == "rotation") 
            && topicData.find("Wheels") != topicData.end()) {
            ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>(topicData["Wheels"], 1);
            geometry_msgs::Twist stopCmd;
            stopCmd.angular.z = 0;
            velPub.publish(stopCmd);
            ros::Duration(0.1).sleep(); // Give time for message to be sent
        }

        // Cancel goals for robot platform
        if (configParams["platform"] == "robot") {
            std::vector<std::string> controllers;
            
            // Select controllers based on behavior
            if (configParams["behaviour"] == "All") {
                controllers = {
                    topicData["RArm"],
                    topicData["LArm"], 
                    topicData["RHand"],
                    topicData["LHand"],
                    topicData["Leg"]
                };
            }
            else if (configParams["behaviour"] == "hands") {
                controllers = {
                    topicData["RHand"],
                    topicData["LHand"]
                };
            }
            else if (configParams["behaviour"] == "body") {
                controllers = {
                    topicData["RArm"],
                    topicData["LArm"],
                    topicData["Leg"]
                };
            }
            // No controllers to cancel for "rotation" only
            
            // Cancel goals for selected controllers
            for (const auto& topic : controllers) {
                try {
                    if(verboseMode){
                        ROS_INFO("[%s]Cancelling goals for: %s", nodeName.c_str(),topic.c_str());
                    }
                    auto client = createClient(topic);
                    auto state = client->getState();
                    if (state == actionlib::SimpleClientGoalState::ACTIVE ||
                        state == actionlib::SimpleClientGoalState::PENDING) {
                        client->cancelAllGoals();
                       ROS_INFO("[%s]Cancelled active goals on: %s", nodeName.c_str(),topic.c_str());
                    }
                } catch (const std::exception& e) {
                    ROS_WARN("[%s] Failed to cancel goals on %s: %s", nodeName.c_str(),topic.c_str(), e.what());
                }
            }
        }
        // Similar handling for simulator platform
        else if (configParams["platform"] == "simulator") {
            std::vector<std::string> controllers;
            
            if (configParams["behaviour"] == "All" || configParams["behaviour"] == "body") {
                controllers = {
                    topicData["RArm"],
                    topicData["LArm"],
                    topicData["Leg"]
                };
                
                for (const auto& topic : controllers) {
                    try {
                        ROS_INFO("[%s] Cancelling goals for: %s", nodeName.c_str(),topic.c_str());
                        auto client = createClient(topic);
                        auto state = client->getState();
                        if (state == actionlib::SimpleClientGoalState::ACTIVE ||
                            state == actionlib::SimpleClientGoalState::PENDING) {
                            client->cancelAllGoals();
                            ROS_INFO("[%s] Cancelled active goals on: %s", nodeName.c_str(),topic.c_str());
                        }
                    } catch (const std::exception& e) {
                        ROS_WARN("[%s]: Failed to cancel goals on %s: %s", nodeName.c_str(),topic.c_str(), e.what());
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] cleanupAndCancelGoals: Error during cleanup: %s", nodeName.c_str(),e.what());
    }
    
    // ROS_INFO("Cleanup and goal cancellation complete");
}

/**
 * @brief Handles shutdown signal (Ctrl+C)
 * @param sig Signal number
 */
void sigintHandler(int sig) {
    try {
        // First stop all behaviors by setting isActive to false
        isActive = false;
        
        // Then stop heartbeat
        stopHeartbeat();
        ROS_ERROR("%s: Node has been killed/terminated unexpectedly", nodeName.c_str());
        
        // Now cleanup goals
        ros::NodeHandle nh;
        cleanupAndCancelGoals(nh);
        
        // Force an immediate shutdown without any sleeps or spinOnce
        ros::shutdown();
        _exit(0);  // Use _exit instead of exit for immediate termination
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] sigintHandler: Error during shutdown cleanup: %s", nodeName.c_str(), e.what());
        _exit(1);  // Force exit on error
    }
}


/**
 * ROS service handler for enabling/disabling the animate behaviour
 * @param req Service request containing desired state
 * @param res Service response indicating success/failure ("1" for enabled, "0" for disabled)
 * @return Always returns true to indicate service was handled
 */
bool setActivation(cssr_system::setActivation::Request &req, cssr_system::setActivation::Response &res) {
    if (req.state == "enabled") {
        // Reload configuration when enabling
        std::string configFilename = "animateBehaviourConfiguration.ini";
        std::string configPath = ros::package::getPath(ROS_PACKAGE_NAME) + "/animateBehaviour/config/" + configFilename;
        
        try {
            loadConfiguration(configPath);
            if (verboseMode) {
                ROS_INFO("[%s]Configuration reloaded successfully", nodeName.c_str());
            }
            logToFile("Configuration reloaded on enable");
            
            isActive = true;
            res.success = "1";
            if (verboseMode) {
                ROS_INFO("[%s]Animate behaviour enabled.",nodeName.c_str());
            }
            logToFile("Animate behaviour enabled.");
        } catch (const std::exception& e) {
            ROS_ERROR("%s: Failed to reload configuration: %s", nodeName.c_str(), e.what());
            logToFile("Failed to reload configuration: " + std::string(e.what()));
            res.success = "0";
            return true;
        }
    } else if (req.state == "disabled") {
        isActive = false;  
        res.success = "0"; 
        ROS_WARN("%s Node is inactive. Waiting to be enabled via service call: 'rosservice call /animateBehaviour/setActivation \"state: 'enabled'\"'", nodeName.c_str());
        logToFile("Animate behaviour disabled.");
        
        // Cancel all active goals when disabling
        try {
            ros::NodeHandle nh;
            cleanupAndCancelGoals(nh);
            ros::Duration(0.1).sleep();
            if (verboseMode) {
                ROS_INFO("[%s]Animate behaviour disabled and goals cancelled.",nodeName.c_str());
                ROS_INFO("[%s]To run the node again, please enable it using the service.",nodeName.c_str()); 
            }
            logToFile("Animate behaviour disabled and goals cancelled.");
        } catch (const std::exception& e) {
            ROS_ERROR("[%s]setActivation: Error during disable cleanup: %s", nodeName.c_str(),e.what());
            logToFile("Error during disable cleanup: " + std::string(e.what()));
        }
    } else {
        res.success = "0";  
        ROS_WARN("[%s]setActivation: Invalid state requested: %s. Use 'enabled' or 'disabled'.", nodeName.c_str(),req.state.c_str());
        logToFile("Invalid state requested: " + req.state + ". Use 'enabled' or 'disabled'.");
    }
    return true;
}

/* Utility Functions for data conversion */


/**
 * Converts a vector of doubles to a string representation
 * 
 * @param vec Vector to be converted
 * @return String representation of the vector
 */
std::string vectorToString(const std::vector<double>& vec) {
    std::ostringstream oss;    // Create string stream for building output
    oss << "[";               // Start with opening bracket
    
    // Iterate through each element in the vector
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];        // Add current element to string
        if (i < vec.size() - 1) {
            oss << ", ";      // Add comma and space if not the last element
        }
    }
    
    oss << "]";              // Close with ending bracket
    return oss.str();        // Convert string stream to string and return
}

/**
 * @brief Converts a 2D vector of doubles to a string format [[x1,y1], [x2,y2],...].
 * @param vec2d Input 2D vector
 * @return String representation of the vector
 */
std::string vector2dToString(const std::vector<std::vector<double>>& vec2d) {
    std::ostringstream oss;    // Create string stream for building output
    
    oss << "[";               // Start with opening bracket
    
    // Iterate through each inner vector
    for (size_t i = 0; i < vec2d.size(); ++i) {
        // Convert current inner vector to string using vectorToString helper
        oss << vectorToString(vec2d[i]);
        
        if (i < vec2d.size() - 1) {
            oss << ", ";      // Add comma and space if not the last vector
        }
    }
    
    oss << "]";              // Close with ending bracket
    
    return oss.str();        // Convert string stream to string and return
}

/**
 * @brief Converts vector of strings to format ["str1", "str2",...].
 * @param vec Input string vector
 * @return String representation of the vector
 */
std::string vectorOfStringsToString(const std::vector<std::string>& vec) {
    std::ostringstream oss;    // Create string stream for building output
    
    oss << "[";               // Start with opening bracket
    
    // Iterate through each string in the vector
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << "\"" << vec[i] << "\"";  // Add quotes around each string
        
        if (i < vec.size() - 1) {
            oss << ", ";      // Add comma and space if not the last string
        }
    }
    
    oss << "]";              // Close with ending bracket
    
    return oss.str();        // Convert string stream to string and return
}

/**
 * @brief Removes leading and trailing whitespace from a string.
 * @param s String to be trimmed, modified in place
 */
void trim(std::string &s) {
    /* Remove leading whitespace */
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), 
        [](unsigned char ch) {
            return !std::isspace(ch);
        }
    ));
    
    /* Remove trailing whitespace */
    s.erase(std::find_if(s.rbegin(), s.rend(),
        [](unsigned char ch) {
            return !std::isspace(ch);
        }
    ).base(), s.end());
}

/**
 * @brief Loads and parses configuration from specified file.
 * @param filename Path to configuration file
 * @return None
 * 
 * Reads key-value pairs from file, populates configParams map.
 * Sets verboseMode and validates required parameters.
 */
void loadConfiguration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("[%s]: loadConfiguration: Unable to open the config file: %s", nodeName.c_str(), filename.c_str());
        logToFile("Unable to open the config file: " + filename);
        return;
    }
    
    logToFile("Configuration file opened successfully: " + filename);
    
    /* Clear existing configuration */
    configParams.clear();
    
    std::string line;
    while (std::getline(file, line)) {
        /* Remove carriage returns */
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        trim(line);
        
        /* Skip empty lines */
        if (line.empty()) {
            continue;
        }
        
        /* Parse key-value pairs */
        size_t delimiterPos = line.find_first_of("\t ");
        if (delimiterPos != std::string::npos) {
            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 1);
            
            trim(key);
            trim(value);
            
            configParams[key] = value;

        }
    }
    
    file.close();
    
    /* Handle missing behaviour parameter */
    if (configParams.find("behaviour") == configParams.end()) {
        configParams["behaviour"] = "";
        ROS_INFO("[%s]Behaviour not found in configuration. Added with empty value.",nodeName.c_str());
    }
    
    /* Set verboseMode flag */
    verboseMode = (configParams.find("verboseMode") != configParams.end() && 
                  configParams["verboseMode"] == "true");
        /* Validate platform configuration */
        if (configParams.find("platform") == configParams.end()) {
            ROS_ERROR("[%s]: loadConfiguration: Platform not found in configuration file",nodeName.c_str());
        }
    
    for (const auto& pair : configParams) {
        logToFile("Config: " + pair.first + " = " + pair.second);
    }
} 

/**
 * @brief Loads platform-specific topic mapping data from file.
 * @param platform Target platform ("robot" or "simulator")
 */
void loadDataBasedOnPlatform(const std::string& platform) {
    if (verboseMode) {
        ROS_INFO("[%s]---------- Loading Platform Data Start ----------",nodeName.c_str());
    }
    
    std::string base_path = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string dataFilePath = base_path + "/animateBehaviour/data/";
    
    if (verboseMode) {
        ROS_INFO("[%s]Data directory path: %s", nodeName.c_str(), dataFilePath.c_str());
    }
    
    /* Select the appropriate data file based on the platform */
    if (platform == "robot") {
        dataFilePath += "pepperTopics.dat";
    } else if (platform == "simulator") {
        dataFilePath += "simulatorTopics.dat";
    } else {
        ROS_ERROR("[%s]: loadDataBasedOnPlatform: Unknown platform: %s" ,nodeName.c_str(),platform.c_str());
        return;
    }
    
    std::ifstream file(dataFilePath);
    if (!file.is_open()) {
        ROS_ERROR("[%s]:loadDataBasedOnPlatform: Unable to open the data file: %s", nodeName.c_str(), dataFilePath.c_str());
        return;
    }
    
    if (verboseMode) {
        ROS_INFO("[%s]: Successfully opened data file",nodeName.c_str());
    }
    
    std::string line;
    int lineCount = 0;
    while (std::getline(file, line)) {
        lineCount++;
        
        std::istringstream is_line(line);
        std::string key, value;
        
        /* Read the key up to the first whitespace and the rest of the line as value */
        if (is_line >> std::noskipws >> key >> std::skipws && std::getline(is_line >> std::ws, value)) {
            /* Correct the key by removing any trailing spaces */
            auto firstSpace = key.find(' ');
            if (firstSpace != std::string::npos) {
                key.erase(firstSpace);
            }
            
            trim(key);
            trim(value);
            
            topicData[key] = value;
        }
    }
    
    /* Log all loaded data */
    for (const auto& pair : topicData) {
        logToFile("Key: " + pair.first + ", Value: " + pair.second);
    }
    
    file.close();
    
    if (verboseMode) {
        ROS_INFO("[%s]---------- Loading Platform Data End ----------",nodeName.c_str());
    }
    
    logToFile("Data loaded successfully for platform: " + platform);
}


/**
 * @brief Creates action client with exponential backoff connection.
 * @param topicName ROS topic name for action client
 * @return Pointer to created action client
 * @throws std::runtime_error if connection fails after max attempts
 */
ControlClientPtr createClient(const std::string& topicName) {
    if (verboseMode) {
        ROS_INFO("[%s]Creating action client for topic: %s", nodeName.c_str(),topicName.c_str());
    }
    ControlClientPtr actionClient(new ControlClient(topicName, true));
    /* Exponential backoff parameters */
    const int maxAttempts = 5;
    double waitTime = 5.0;                  // Initial wait time in seconds
    const double backoffMultiplier = 2.0;

    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        if (verboseMode) {
            ROS_INFO("[%s]Waiting for action server. Attempt %d/%d", nodeName.c_str(),attempt, maxAttempts);        
        }
        if (actionClient->waitForServer(ros::Duration(waitTime))) {
            if (verboseMode) {
                ROS_INFO("[%s]Action server connected after %d attempt(s)", nodeName.c_str(),attempt);
            }
            return actionClient;
        } 
        if (verboseMode){
            ROS_WARN("[%s]:Connection attempt %d failed. Retrying...", nodeName.c_str(),attempt);
        }
        waitTime *= backoffMultiplier;
    }
    ROS_ERROR("[%s]:createClient: Connection failed after %d attempts: %s", nodeName.c_str(),maxAttempts, topicName.c_str());

    throw std::runtime_error("[" + nodeName + "] createClient: Failed to connect to action server: " + topicName);
}

/**
 * @brief Generates random position within specified range.
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return Random value between min and max
 */
double generateRandomPosition(double min, double max) {
    std::random_device rd;                                      // Create random device
    std::mt19937 engine(rd());                                 // Initialize random engine
    std::uniform_real_distribution<> distribution(min, max);   // Create distribution
    
    return distribution(engine);
}

/**
 * @brief Initialize random number generator if not already done.
 */
void initRandomSeed() {
    static bool initialized = false;
    
    if (!initialized) {
        std::srand(std::time(nullptr));
        initialized = true;
    }
}

/**
 * @brief Check if this is the first run of the program.
 * @return true if first run, false otherwise
 */
bool isFirstRun() {
    std::ifstream infile(FLAG_FILE_PATH);
    bool firstRun = !infile.good();
    infile.close();
    return firstRun;
}

/**
 * @brief Update flag file to indicate program has run.
 */
void updateFirstRunFlag() {
    std::ofstream outfile("first_run_flag.txt");
    outfile << "false";
    outfile.close();
}


/**
 * @brief Parse comma-separated percentage values into vector.
 * @param percentagesStr String of comma-separated percentages
 * @return Vector of parsed percentage values
 */
std::vector<double> parsePercentages(const std::string& percentagesStr) {
    std::vector<double> percentages;
    std::istringstream percentagesStream(percentagesStr);
    std::string percentage;
    
    while (std::getline(percentagesStream, percentage, ',')) {
        try {
            percentages.push_back(std::stod(percentage));
            
            // if (verboseMode) {
            //     ROS_INFO("Parsed percentage: %s", percentage.c_str());
            // }
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("[%s]:Invalid percentage format: %s", nodeName.c_str(), percentage.c_str());
        } catch (const std::out_of_range& e) {
            ROS_ERROR("[%s]:Percentage out of range: %s", nodeName.c_str(), percentage.c_str());
        }
    }
    
    return percentages;
}

/**
 * @brief Calculates target positions for robot movement.
 * 
 * @param homePosition Current/home position of robot joints
 * @param maxPosition Maximum allowable position for joints
 * @param minPosition Minimum allowable position for joints
 * @param jointType Joint type ("arm", "leg", or "hand")
 * @param count Number of positions to generate
 * @return Vector of calculated target positions
 */
std::vector<std::vector<double>> calculateTargetPosition(
    const std::vector<double>& homePosition,
    const std::vector<double>& maxPosition,
    const std::vector<double>& minPosition,
    const std::string& jointType,
    int count) {
    
    /* Initialize variables */
    std::vector<std::vector<double>> targetPositions;
    double selectedRange = std::stod(configParams["selectedRange"]);
    double maxCount = std::stod(configParams["numPoints"]);
    int repeatFactor = std::stod(configParams["legRepeatFactor"]);

    // if (verboseMode) {
    //     ROS_INFO("-------------- Starting Position Calculation --------------");
    //     ROS_INFO("Joint Type: %s", jointType.c_str());
    //     ROS_INFO("Selected Range: %.2f", selectedRange);
    //     ROS_INFO("Max Count: %.2f", maxCount);
    //     ROS_INFO("Repeat Factor: %d", repeatFactor);
    // }

    try {
        /* Determine maximum range based on joint type */
        std::vector<double> maximumRange;
        
        if (jointType == "arm") {
            if (configParams.find("armMaximumRange") == configParams.end()) {
                throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: armMaximumRange not found in configuration");
            }
            maximumRange = parsePercentages(configParams["armMaximumRange"]);
            if (verboseMode) {
                ROS_INFO("[%s]Loaded arm maximum range parameters" ,nodeName.c_str());
            }
        } 
        else if (jointType == "leg") {
            if (configParams.find("legMaximumRange") == configParams.end()) {
                throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: legMaximumRange not found in configuration");
            }
            maximumRange = parsePercentages(configParams["legMaximumRange"]);
            if (verboseMode) {
                ROS_INFO("[%s]:Loaded leg maximum range parameters" ,nodeName.c_str());
            }
        } 
        else if (jointType == "hand") {
            if (configParams.find("handMaximumRange") == configParams.end()) {
                throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: handMaximumRange not found in configuration");
            }
            maximumRange.push_back(std::stod(configParams["handMaximumRange"]));
            if (verboseMode) {
                ROS_INFO("[%s]Loaded hand maximum range parameter" ,nodeName.c_str());
            }
        } 
        else {
           throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: Unknown joint type: " + jointType);
        }

        /* Validate configuration and inputs */
        if (configParams.find("selectedRange") == configParams.end()) {
            throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: selectedRange not found in configuration");
        }

        if (homePosition.size() != maxPosition.size() || 
            homePosition.size() != minPosition.size()) {
            throw std::runtime_error("[" + nodeName + "] calculateTargetPosition: Mismatch in vector sizes for home, max, and min positions");
        }

        /* Handle leg joint type specially */
        if (jointType == "leg") {
            int numJoints = homePosition.size();    
            int chunkSize = maxCount / repeatFactor;  
            int remainingPositions = static_cast<int>(maxCount) % static_cast<int>(chunkSize);

            if (verboseMode) {
                ROS_INFO("[%s]Leg Movement Parameters:" ,nodeName.c_str());
                ROS_INFO("[%s]  Number of Joints: %d", nodeName.c_str(), numJoints);
                ROS_INFO("[%s]  Chunk Size: %d", nodeName.c_str(),chunkSize);
                ROS_INFO("[%s]  Remaining Positions: %d", nodeName.c_str(), remainingPositions);
            }
            
            if (count > chunkSize) {
                std::stringstream errorMsg;
                errorMsg << "[" << nodeName.c_str() << "] calculateTargetPosition: Count (" << count << ") exceeds chunk size (" << chunkSize 
                        << "). Adjust numPointsLeg in config.";
                throw std::runtime_error(errorMsg.str());
            }

            targetPositions.clear();

            /* Generate positions for each chunk */
            for (int i = 0; i < repeatFactor; ++i) {
                if (verboseMode) {
                    ROS_INFO("[%s]Processing chunk %d/%d", nodeName.c_str(), i + 1, repeatFactor);
                }

                /* Generate random positions for each joint */
                for (size_t k = 0; k < count; ++k) {
                    std::vector<double> singleTargetPosition;
                    
                    for (size_t j = 0; j < numJoints; ++j) {
                        if (j >= maximumRange.size()) {
                            throw std::runtime_error("[" + nodeName + "] Insufficient maximum range values for " + jointType);
                        }

                        /* Calculate position bounds */
                        double maxRange = maximumRange[j];
                        double fullRange = maxPosition[j] - minPosition[j];
                        double maximumRangeOffset = fullRange * maxRange;
                        double selectedRangeOffset = (maximumRangeOffset * selectedRange) / 2.0;

                        double tempPositionMax = std::min(homePosition[j] + selectedRangeOffset, maxPosition[j]);
                        double tempPositionMin = std::max(homePosition[j] - selectedRangeOffset, minPosition[j]);

                        /* Generate and store random position */
                        double randomPosition = generateRandomPosition(tempPositionMin, tempPositionMax);
                        singleTargetPosition.push_back(randomPosition);
                    }

                    targetPositions.push_back(singleTargetPosition);
                }

                /* Fill remaining chunk positions */
                while (targetPositions.size() < (i + 1) * chunkSize) {
                    targetPositions.push_back(homePosition);
                }
            }

            /* Handle remaining positions */
            for (int i = 0; i < remainingPositions; ++i) {
                targetPositions.push_back(homePosition);
            }

            /* Validate final position count */
            if (targetPositions.size() != maxCount) {
                if (verboseMode) {
                    ROS_INFO("[%s]:Generated %zu positions, expected %f", nodeName.c_str(),targetPositions.size(), maxCount);
                }
                throw std::runtime_error("[" + nodeName + "] Generated position count mismatch");
            }
        }
        /* Handle arm and hand joints */
        else {
            if (verboseMode) {
                ROS_INFO("[%s]:Generating %f positions for %s", nodeName.c_str(), maxCount, jointType.c_str());
            }

            for (int i = 0; i < maxCount; ++i) {
                std::vector<double> singleTargetPosition;
                
                for (size_t j = 0; j < homePosition.size(); ++j) {
                    if (j >= maximumRange.size()) {
                       throw std::runtime_error("[" + nodeName + "] Insufficient maximum range values for " + jointType);
                    }

                    /* Calculate position bounds */
                    double maxRange = maximumRange[j];
                    double fullRange = maxPosition[j] - minPosition[j];
                    double maximumRangeOffset = fullRange * maxRange;
                    double selectedRangeOffset = (maximumRangeOffset * selectedRange) / 2.0;

                    double tempPositionMax = std::min(homePosition[j] + selectedRangeOffset, maxPosition[j]);
                    double tempPositionMin = std::max(homePosition[j] - selectedRangeOffset, minPosition[j]);

                    /* Generate and store random position */
                    double randomPosition = generateRandomPosition(tempPositionMin, tempPositionMax);
                    singleTargetPosition.push_back(randomPosition);
                }
                
                targetPositions.push_back(singleTargetPosition);
            }
        }

        return targetPositions;  

    } catch (const std::exception& e) {
        ROS_ERROR("[%s]:Position calculation failed: %s", nodeName.c_str(), e.what());
        return std::vector<std::vector<double>>(homePosition.size(), std::vector<double>(count, 0.0));
    }
}

/**
 * @brief Moves robot to specified position using action server.
 * 
 * @param client Action client for sending goals
 * @param jointNames Names of joints to move
 * @param positionName Identifier for target position
 * @param positions Target joint positions
 */
void moveToPosition(ControlClientPtr& client, 
                   const std::vector<std::string>& jointNames,
                   const std::string& positionName, 
                   std::vector<double> positions) {
    
    if (verboseMode) {
        ROS_INFO("[%s]:-------------- Moving to Target Position --------------",nodeName.c_str());
        ROS_INFO("[%s]:Target: %s", nodeName.c_str(),positionName.c_str());
        ROS_INFO("[%s]:Number of joints: %zu", nodeName.c_str(), jointNames.size());
    }
    
    /* Initialize trajectory goal */
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    
    /* Configure trajectory point */
    trajectory.points.resize(1);
    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(0.2);

    if (verboseMode) {
        ROS_INFO("[%s]Joint positions:", nodeName.c_str());
        for (size_t i = 0; i < jointNames.size(); ++i) {
            ROS_INFO("[%s]  %s: %.4f", nodeName.c_str(), jointNames[i].c_str(), positions[i]);
        }
        ROS_INFO("[%s]Sending goal to action server...", nodeName.c_str());
    }

    /* Send goal and wait for result */
    client->sendGoal(goal);
    const double TIMEOUT_DURATION = 10.0;
    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(TIMEOUT_DURATION));
    
    /* Handle action result */
    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (verboseMode) {
                ROS_INFO("[%s]:Movement succeeded", nodeName.c_str());
                ROS_INFO("[%s]:Final state: %s", nodeName.c_str(), state.toString().c_str());
            }
        } else {
            if (verboseMode){
                ROS_WARN("[%s]:Movement failed to %s position", nodeName.c_str(), positionName.c_str());
                ROS_WARN("[%s]:Final state: %s", nodeName.c_str(), state.toString().c_str());
            }
        }
    } else {
        if(verboseMode){
            ROS_WARN("[%s]Movement timed out after %.1f seconds", nodeName.c_str(), TIMEOUT_DURATION);
        }
    }

    if (verboseMode) {
        ROS_INFO("[%s]-------------- Movement To Target Postion Complete --------------", nodeName.c_str());
    }
}

/**
 * @brief Executes biologically-inspired joint movement trajectory.
 * 
 * @param client Action client for sending goals
 * @param jointNames Names of joints to control
 * @param positionName Target position identifier
 * @param positions Vector of target positions for each joint
 * @param durations Duration for each trajectory point
 */
void moveToListOfPositions(ControlClientPtr& client, 
                            const std::vector<std::string>& jointNames,
                            const std::string& positionName, 
                            const std::vector<std::vector<double>>& positions, 
                            const std::vector<double>& durations) {
    
    if (verboseMode) {
        ROS_INFO("[%s]-------------- Starting Biological Movement --------------", nodeName.c_str());
        // ROS_INFO("Target: %s", positionName.c_str());
        // ROS_INFO("Number of joints: %zu", jointNames.size());
        // ROS_INFO("Number of waypoints: %zu", positions.size());
    }
    
    /* Initialize trajectory goal */
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;

    /* Configure trajectory points */
    const size_t numPoints = positions.size();
    trajectory.points.resize(numPoints);
    double cumulativeTime = 0.0;
    
    /* Populate trajectory points */
    for (size_t i = 0; i < numPoints; ++i) {
        /* Validate point data */
        if (positions[i].size() != jointNames.size()) {
            ROS_ERROR("[%s]moveToListOfPositions: Joint count mismatch at point %zu", nodeName.c_str(), i);
            ROS_ERROR("[%s]Expected %zu joints, got %zu", nodeName.c_str(), jointNames.size(), positions[i].size());
            return;
        }

        /* Configure trajectory point */
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions[i];
        
        cumulativeTime += durations[i];
        point.time_from_start = ros::Duration(cumulativeTime);
        
        trajectory.points[i] = point;
    }

    /* Send trajectory to action server */
    client->sendGoal(goal);
    const double BUFFER_TIME = 0.5;
    const double totalDuration = cumulativeTime + BUFFER_TIME;
    
    /* Monitor execution */
    const double CHECK_RATE = 10.0;  // Hz
    ros::Rate rate(CHECK_RATE);
    
    while (!client->getState().isDone()) {
        ros::spinOnce();
        
        /* Check for cancellation */
        if (!isActive || !ros::ok()) {
            // ROS_WARN("rotationBaseShift: Movement interrupted - cancelling trajectory");
            auto currentState = client->getState();
            if (currentState == actionlib::SimpleClientGoalState::ACTIVE || 
                currentState == actionlib::SimpleClientGoalState::PENDING) {
                client->cancelGoal();
            } 
            return;
        }
        
        rate.sleep();
    }

    /* Check final result */
    actionlib::SimpleClientGoalState state = client->getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (verboseMode) {
            ROS_INFO("[%s]Movement to %s completed successfully", nodeName.c_str(), positionName.c_str());
            ROS_INFO("[%s]Final state: %s", nodeName.c_str(), state.toString().c_str());
        }
    } else {
        if (verboseMode) {
            ROS_WARN("[%s]:Movement to %s failed", nodeName.c_str(), positionName.c_str());
            ROS_WARN("[%s]:Final state: %s", nodeName.c_str(), state.toString().c_str());
        }
    }

    if (verboseMode) {
        ROS_INFO("[%s]-------------- Biological Movement Complete --------------", nodeName.c_str());
    }
}

/**
 * @brief Controls animated movement of robot's left arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftArmTopic Topic for left arm control
 * @param resetPosition Flag to reset to home position
 */
void lArm(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]------------------------------LEFT ARM ANIMATION START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]Topic: %s", nodeName.c_str(), leftArmTopic.c_str());
    }
    
    /* Initialize action client */
    ControlClientPtr leftArmClient = createClient(leftArmTopic); 

    /* Define joint position limits */                  
    const std::vector<double> minPosition = {
        -2.0857,  0.0087, -1.5620, -2.0857, -1.8239     // Minimum positions
    };  
    const std::vector<double> maxPosition = {
        2.0857, 1.5620, -0.0087,  2.0857,  1.8239       // Maximum positions
    };    
    const std::vector<double> homePosition = {
        1.7625, 0.09970, -0.1334, -1.7150, 0.06592      // Home positions
    };   
    const std::vector<double> homePositionT = {
        1.5625, 0.09970, -0.3434, -1.7150, 0.06592      // Modified home positions
    };
    
    /* Initialize movement parameters */
    static std::vector<double> currentPosition;           // Tracks arm state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stod(configParams["numPoints"]); 


    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePositionT;
        moveToPosition(leftArmClient, lArmJointNames, "Home Position", homePosition);
    }

    /* Generate movement trajectory */
    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePositionT, maxPosition, minPosition, "arm", count);
    
    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    if (verboseMode) {
        ROS_INFO("[%s]Randomly generated positions are: %zu", nodeName.c_str(), randomPositions.size());
        ROS_INFO("[%s]  Duration per movement: %.2f", nodeName.c_str(), duration);
    }
    
    /* Log movement data */
    logToFile("----------[START LEFT ARM ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the left arm is " + leftArmTopic);
    logToFile("The joint names for the left arm are: " + 
              vectorOfStringsToString(lArmJointNames));
    logToFile("The home position of the left arm is " + 
              vectorToString(homePosition));
    logToFile("The random position of the left arm are");
    
    const size_t loopLimit = std::min(randomPositions.size(), 
                                     static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + 
              std::to_string(loopLimit));

    /* Log initial positions */
    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(randomPositions[i])); 
    }

    logToFile("----------[END LEFT ARM ANIMATE MOVEMENT]-----------");
    
    moveToListOfPositions(leftArmClient, lArmJointNames, "Animate Behavior", 
                            randomPositions, fixedDurations);
    
    /* Update final position */
    if (!randomPositions.empty()) {
        currentPosition = randomPositions.back();  
        if (verboseMode) {
            ROS_INFO("[%s]------------------------------ LEFT ARM ANIMATION COMPLETE ------------------------------", nodeName.c_str());
        }
    }
}

/**
 * @brief Controls flexible movement of robot's left arm wrist.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftArmTopic Topic for left arm control
 * @param resetPosition Flag to reset to home position
 */
void lArml(ros::NodeHandle& nh, std::string leftArmTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ LEFT ARM FLEXI MOVEMENT START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]:Topic: %s", nodeName.c_str(), leftArmTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr leftArmClient = createClient(leftArmTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {
        -2.0857, 0.0087, -1.5620, -2.0857, -1.8239      // Minimum positions
    };
    const std::vector<double> maxPosition = {
        2.0857, 1.5620, -0.0087, 2.0857, 1.8239         // Maximum positions
    };
    const std::vector<double> homePosition = {
        1.7625, 0.09970, -0.1334, -1.7150, 0.06592      // Home positions
    };
    const std::vector<double> homePositionT = {
        1.5625, 0.09970, -0.3434, -1.7150, 0.06592      // Modified home positions
    };

    /* Initialize movement parameters */
    static std::vector<double> currentPosition;           // Persistent state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stoi(configParams["numPoints"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePositionT;
        moveToPosition(leftArmClient, lArmJointNames, "Home Position", currentPosition);
    }
    
    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePositionT, maxPosition, minPosition, "arm", count);
    
    std::vector<std::vector<double>> updatedPositions;
    for (size_t i = 0; i < randomPositions.size(); ++i) {
        /* Keep most joints at home position, only move wrist */
        std::vector<double> updatedPosition = {
            homePositionT[0],          // Fixed shoulder pitch
            homePositionT[1],          // Fixed shoulder roll
            randomPositions[i][2],     // Variable elbow roll
            homePositionT[3],          // Fixed elbow yaw
            randomPositions[i][4]      // Variable wrist yaw
        };
        updatedPositions.push_back(updatedPosition);
    }

    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    
    if (verboseMode) {
        ROS_INFO("[%s]:  Total positions: %zu", nodeName.c_str(), updatedPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str() ,duration);
    }

    /* Log movement data */
    logToFile("----------[START LEFT ARM FLEXI MOVEMENT]-----------");
    logToFile("The topic name for the left arm is " + leftArmTopic);
    logToFile("The joint names for the left arm are: " + 
              vectorOfStringsToString(lArmJointNames));
    logToFile("The home position of the left arm is " + 
              vectorToString(homePositionT));
    logToFile("The random position of the left arm are");
    const size_t loopLimit = std::min(randomPositions.size(), 
                                     static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + 
              std::to_string(loopLimit));
    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(updatedPositions[i]));
    }

    logToFile("----------[END LEFT ARM FLEXI MOVEMENT]-----------");
 
    moveToListOfPositions(leftArmClient, lArmJointNames, "Animate Behavior", 
                            updatedPositions, fixedDurations);

    /* Update final position */
    if (!updatedPositions.empty()) {
        currentPosition = updatedPositions.back();
        if (verboseMode) {
            ROS_INFO("[%s]:------------------------------ LEFT ARM FLEXI MOVEMENT COMPLETE ------------------------------", nodeName.c_str());
        }
    }
}

/**
 * @brief Controls animated movement of robot's left hand.
 * 
 * @param nh ROS NodeHandle for communication
 * @param leftHandTopic Topic for left hand control
 * @param resetPosition Flag to reset to home position
 */
void lHand(ros::NodeHandle& nh, std::string leftHandTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ LEFT HAND ANIMATION START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]:Topic: %s", nodeName.c_str(), leftHandTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr leftHandClient = createClient(leftHandTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {0.0};       // Minimum position
    const std::vector<double> maxPosition = {1.0};       // Maximum position
    const std::vector<double> homePosition = {0.6695};   // Home position

    /* Initialize movement parameters */
    static std::vector<double> currentPosition;          // Persistent state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stod(configParams["numPoints"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(leftHandClient, lhandJointNames, "Home Position", homePosition);
    }
    
    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePosition, maxPosition, minPosition, "hand", count);

    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    
    if (verboseMode) {
        ROS_INFO("[%s]:  Total random positions: %zu", nodeName.c_str(), randomPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str(), duration);
    }

    /* Log movement data */
    logToFile("----------[START LEFT HAND ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the left hand is " + leftHandTopic);
    logToFile("The joint names for the left hand are: " + 
              vectorOfStringsToString(lhandJointNames));
    logToFile("The home position of the left hand is " + 
              vectorToString(homePosition));
    logToFile("The random position of the left hand are");
    const size_t loopLimit = std::min(randomPositions.size(), 
                                     static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + 
              std::to_string(loopLimit));

    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(randomPositions[i]));
    }
    logToFile("----------[END LEFT HAND ANIMATE MOVEMENT]-----------");

    
    moveToListOfPositions(leftHandClient, lhandJointNames, "Animate Behavior", 
                            randomPositions, fixedDurations);

    /* Update final position */
    if (!randomPositions.empty()) {
        currentPosition = randomPositions.back();
        if (verboseMode) {
            ROS_INFO("[%s]:------------------------------ LEFT HAND ANIMATION COMPLETE ------------------------------", nodeName.c_str());
        }
    }
}

/**
 * @brief Controls animated movement of robot's right arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param rightArmTopic Topic for right arm control
 * @param resetPosition Flag to reset to home position
 */
void rArm(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ RIGHT ARM ANIMATION START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]:Topic: %s", nodeName.c_str(), rightArmTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr rightArmClient = createClient(rightArmTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {
        -2.0857, -1.5620,  0.0087, -2.0857, -1.5620    // Minimum positions
    };
    const std::vector<double> maxPosition = {
        2.0857,  -0.0087,  1.5620,  2.0857,  1.8239    // Maximum positions
    };
    const std::vector<double> homePosition = {
        1.7410,  -0.09664, 0.09664, 1.6981, -0.05679   // Home positions
    };
    const std::vector<double> homePositionT = {
        1.5410,  -0.09664, 0.30664, 1.6981, -0.05679   // Modified home positions
    };

    /* Initialize movement parameters */
    static std::vector<double> currentPosition;          // Persistent state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stoi(configParams["numPoints"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePositionT;
        moveToPosition(rightArmClient, rArmJointNames, "Home Position", currentPosition);
    }
    
    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePositionT, maxPosition, minPosition, "arm", count);

    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    
    if (verboseMode) {
        ROS_INFO("[%s]:  Total random positions: %zu", nodeName.c_str(), randomPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str(), duration);
    }

    /* Log movement data */
    logToFile("----------[START RIGHT ARM ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the right arm is " + rightArmTopic);
    logToFile("The joint names for the right arm are: " + 
              vectorOfStringsToString(rArmJointNames));
    logToFile("The home position of the right arm is " + 
              vectorToString(homePosition));
    logToFile("The random position of the right arm are");
    const size_t loopLimit = std::min(randomPositions.size(), 
                                     static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + 
              std::to_string(loopLimit));

    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(randomPositions[i]));
    }

    logToFile("----------[END RIGHT ARM ANIMATE MOVEMENT]-----------");
    
    moveToListOfPositions(rightArmClient, rArmJointNames, "Animate Behavior", 
                            randomPositions, fixedDurations);

    /* Update final position */
    if (!randomPositions.empty()) {
        currentPosition = randomPositions.back();
        if (verboseMode) {
            ROS_INFO("[%s]:------------------------------ RIGHT ARM ANIMATION COMPLETE------------------------------", nodeName.c_str());
        }
    }
}

/**
 * @brief Controls flexible wrist movement of robot's right arm.
 * 
 * @param nh ROS NodeHandle for communication
 * @param rightArmTopic Topic for right arm control
 * @param resetPosition Flag to reset to home position
 */
void rArml(ros::NodeHandle& nh, std::string rightArmTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ RIGHT ARM FLEXI MOVEMENT START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]: Topic: %s", nodeName.c_str(), rightArmTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr rightArmClient = createClient(rightArmTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {
        -2.0857, -1.5620,  0.0087, -2.0857, -1.5620    // Minimum positions
    };
    const std::vector<double> maxPosition = {
        2.0857,  -0.0087,  1.5620,  2.0857,  1.8239    // Maximum positions
    };
    const std::vector<double> homePosition = {
        1.7410,  -0.09664, 0.09664, 1.6981, -0.05679   // Home positions
    };
    const std::vector<double> homePositionT = {
        1.5410,  -0.09664, 0.30664, 1.6981, -0.05679   // Modified home positions
    };

    /* Initialize movement parameters */
    static std::vector<double> currentPosition;          // Persistent state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stoi(configParams["numPoints"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePositionT;
        moveToPosition(rightArmClient, rArmJointNames, "Home Position", currentPosition);
    }
    
    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePositionT, maxPosition, minPosition, "arm", count);
    
    std::vector<std::vector<double>> updatedPositions;
    for (size_t i = 0; i < randomPositions.size(); ++i) {
        /* Keep most joints fixed at home position, only move specific joints */
        std::vector<double> updatedPosition = {
            homePositionT[0],          // Fixed shoulder pitch
            homePositionT[1],          // Fixed shoulder roll
            randomPositions[i][2],     // Variable elbow roll
            homePositionT[3],          // Fixed elbow yaw
            randomPositions[i][4]      // Variable wrist yaw
        };
        updatedPositions.push_back(updatedPosition);
    }

    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    
    if (verboseMode) {
        ROS_INFO("[%s]:  Total ranodm positions: %zu", nodeName.c_str(), updatedPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str(), duration);
    }

    /* Log movement data */
    logToFile("----------[START RIGHT ARM FLEXI MOVEMENT]-----------");
    logToFile("The topic name for the right arm is " + rightArmTopic);
    logToFile("The joint names for the right arm are: " + 
              vectorOfStringsToString(rArmJointNames));
    logToFile("The home position of the right arm is " + 
              vectorToString(homePositionT));
    logToFile("The random position of the right arm are");
    const size_t loopLimit = std::min(randomPositions.size(), 
                                     static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + 
              std::to_string(loopLimit));

    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(updatedPositions[i]));
    }
    logToFile("----------[END RIGHT ARM FLEXI MOVEMENT]-----------");

    /* Biloogical movement */
    moveToListOfPositions(rightArmClient, rArmJointNames, "Animate Behavior", 
                            updatedPositions, fixedDurations);

    /* Update final position */
    if (!updatedPositions.empty()) {
        currentPosition = updatedPositions.back();
    }
    
    if (verboseMode) {
            ROS_INFO("[%s]:------------------------------ RIGHT ARM FLEXI MOVEMENT COMPLETE ------------------------------", nodeName.c_str());
        }
}
/**
 * @brief Animates the right hand of the robot.
 * 
 * This function calculates target positions for the right hand joints and moves the hand
 * to these positions using a biological trajectory. It also generates and displays diagnostic 
 * information if verbose mode is enabled. The function supports resetting to the home position
 * if requested.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param rightHandTopic The ROS topic name for controlling the right hand's movement.
 * @param resetPosition Boolean flag to indicate whether to reset to the home position.
 */
void rHand(ros::NodeHandle& nh, std::string rightHandTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ RIGHT HAND ANIMATE MOVEMENT START ------------------------------", nodeName.c_str());
        ROS_INFO("[%s]:Topic: %s", nodeName.c_str(), rightHandTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr rightHandClient = createClient(rightHandTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {0.0};       // Minimum position for the hand's joint
    const std::vector<double> maxPosition = {1.0};       // Maximum position for the hand's joint
    const std::vector<double> homePosition = {0.66608};  // Home position for the right hand

    /* Initialize movement parameters */
    static std::vector<double> currentPosition;          // Persistent between function calls
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stoi(configParams["numPoints"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePosition;
        moveToPosition(rightHandClient, rhandJointNames, "Home Position", currentPosition);
    }

    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePosition, maxPosition, minPosition, "hand", count);
    
    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
     if (verboseMode) {
        ROS_INFO("[%s]:  Total random positions: %zu", nodeName.c_str(), randomPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str(), duration);
    }

    /* Log movement data - exact format preserved */
    logToFile("----------[START RIGHT HAND ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the right hand is " + rightHandTopic);
    logToFile("The joint names for the right hand are: " + vectorOfStringsToString(rhandJointNames));
    logToFile("The home position of the right hand is " + vectorToString(homePosition));
    logToFile("The random positions of the right hand are:");
    size_t loopLimit = std::min(randomPositions.size(), static_cast<size_t>(20));  
    logToFile("The size of the generated random positions are: " + std::to_string(loopLimit));
    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(randomPositions[i]));
    }

    logToFile("----------[END RIGHT HAND ANIMATE MOVEMENT]-----------");

    moveToListOfPositions(rightHandClient, rhandJointNames, "Animate Behavior", 
                            randomPositions, fixedDurations);

    /* Update final position */
    if (!randomPositions.empty()) {
        currentPosition = randomPositions.back();
    }

    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------[END RIGHT HAND ANIMATE MOVEMENT]-------------------------------", nodeName.c_str());
    }
}

/**
 * @brief Animates the leg of the robot.
 * 
 * This function calculates target positions for the leg joints and moves the leg
 * to the calculated positions. It first moves the leg to the home position if it is the
 * first run or if a reset is explicitly requested. After that, it continuously moves the leg
 * through randomly generated positions within the defined joint limits. The first move
 * is handled using the moveToPosition function, and subsequent moves are performed using
 * the moveToListOfPositions function for smooth and natural movement.
 * 
 * Diagnostic information is logged if verbose mode is enabled, including details of
 * each movement's joint positions and target positions. The function supports resetting
 * the leg's position to a home state and allows continuous movement from the last position.
 * 
 * @param nh ROS NodeHandle to manage communication with ROS.
 * @param legTopic The ROS topic name for controlling the leg's movement.
 * @param resetPosition Boolean flag indicating whether to reset the leg to the home position.
 */
void leg(ros::NodeHandle& nh, std::string legTopic, bool resetPosition) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------[START LEG ANIMATE MOVEMENT]----------------------------------------", nodeName.c_str());
        ROS_INFO("[%s]:Topic: %s", nodeName.c_str(), legTopic.c_str());
    }

    /* Initialize action client */
    ControlClientPtr legClient = createClient(legTopic);

    /* Define joint position limits */
    const std::vector<double> minPosition = {-1.0385, -0.5149, -0.5149};     // Minimum position for each joint
    const std::vector<double> maxPosition = {1.0385, 0.5149, 0.5149};        // Maximum position for each joint
    const std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};   // Home position for the leg
    const std::vector<double> homePositionT = {-0.1107, -0.00766, 0.03221};  // An alternative home position


    /* Initialize movement parameters */
    static std::vector<double> currentPosition;         // Current position to track the leg's state
    const double duration = std::stod(configParams["gestureDuration"]);
    const int count = std::stod(configParams["numPointsLeg"]);

    /* Handle position reset if needed */
    if (currentPosition.empty() || resetPosition) {
        currentPosition = homePositionT;
        moveToPosition(legClient, legJointNames, "Home Position", currentPosition);
    }

    std::vector<std::vector<double>> randomPositions = 
        calculateTargetPosition(homePositionT, maxPosition, minPosition, "leg", count);
    
    /* Configure movement durations */
    std::vector<double> fixedDurations(randomPositions.size(), duration);
    
    if (verboseMode) {
        ROS_INFO("[%s]:  Total positions: %zu", nodeName.c_str(), randomPositions.size());
        ROS_INFO("[%s]:  Duration per movement: %.2f", nodeName.c_str(), duration);
    }

    /* Log movement data - exact format preserved */

    logToFile("----------[START LEG ANIMATE MOVEMENT]-----------");
    logToFile("The topic name for the leg is " + legTopic);
    logToFile("The joint names for the leg are: " + vectorOfStringsToString(legJointNames));
    logToFile("The home position of the leg is " + vectorToString(homePosition));
    logToFile("The random position of the leg are:");
    size_t loopLimit = std::min(randomPositions.size(), static_cast<size_t>(20));
    logToFile("The size of the generated random positions are: " + std::to_string(loopLimit));
    for (size_t i = 0; i < loopLimit; ++i) {
        logToFile(vectorToString(randomPositions[i]));
    }

    logToFile("----------[END LEG ANIMATE MOVEMENT]-----------");


    moveToListOfPositions(legClient, legJointNames, "Animate Behavior", 
                            randomPositions, fixedDurations);

    /* Update final position */
    if (!randomPositions.empty()) {
        currentPosition = randomPositions.back();
    }
    if(verboseMode){
     ROS_INFO("[%s]:------------------------------[END LEG ANIMATE MOVEMENT]------------------------------",nodeName.c_str());
    }
}

/**
 * @brief Calculates an angular velocity in the Z-axis for the robot's base rotation.
 * 
 * This function calculates a target angular velocity within specified ranges. It ensures the
 * angular velocity is within the max and min positions and adjusts based on the 'rotMaximumRange' 
 * and 'selectedRange' from configParams.
 * 
 * @param maxAngularVelocity The maximum allowable angular velocity.
 * @return The calculated angular velocity in the Z-axis.
 */
double calculateAngularVelocityZ(double maxAngularVelocity) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------CALCULATING ANGULAR VELOCITY Z START ------------------------------", nodeName.c_str());
    }

    /* Initialize configuration parameters */
    const double maxRange = std::stod(configParams["rotMaximumRange"]);
    const double selectedRange = std::stod(configParams["selectedRange"]);
    const double homeAngularVelocity = 0.0;

    /* Calculate velocity ranges */
    const double fullRange = maxAngularVelocity - homeAngularVelocity;
    const double maximumRangeOffset = fullRange * maxRange;
    const double selectedRangeOffset = maximumRangeOffset * selectedRange;

    /* Calculate velocity limits */
    const double tempVelocityMax = homeAngularVelocity + selectedRangeOffset;
    const double tempVelocityMin = homeAngularVelocity - selectedRangeOffset;
    /* Generate random velocity within range */
    const double angularVelocityZ = generateRandomPosition(tempVelocityMin, tempVelocityMax);

    if (verboseMode) {
        ROS_INFO("[%s]:Result:", nodeName.c_str());
        ROS_INFO("[%s]:  Calculated Angular Velocity Z: %.4f", nodeName.c_str(), angularVelocityZ);
        ROS_INFO("[%s]:------------------------------ CALCULATING ANGULAR VELOCITY Z COMPLETE ------------------------------", nodeName.c_str());
    }

    return angularVelocityZ;
}

/**
 * @brief Initiates a rotation of the robot base using a calculated angular velocity.
 * 
 * This function performs a sequence of rotations with angular velocities calculated
 * using the calculateAngularVelocityZ function. It alternates between positive and
 * negative velocities with stops in between, completing a specified number of cycles.
 * 
 * @param nh The ROS NodeHandle.
 */
void rotationBaseShift(ros::NodeHandle& nh) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ [START ROTATION BASE SHIFT] ------------------------------", nodeName.c_str());
    }
    logToFile("[START ROTATION BASE SHIFT]");
    
    double maxAngularVelocity = 2.0; 
    std::string cmdVelTopic = topicData["Wheels"]; 
    std::vector<std::string> baseJointNames = {"Wheels"};
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 10);

    geometry_msgs::Twist twist; 
    double rotationDuration = 2.0; 
    std::atomic<bool> cycleComplete(false);  

    logToFile("[START ROTATION BASE SHIFT]");
    logToFile("The joint names for the base are: " + vectorOfStringsToString(baseJointNames));
    logToFile("The home position of the base is " + std::to_string(maxAngularVelocity));
    logToFile("The random position of the angular velocity are:");
    logToFile("The size of the generated random positions are: " + std::to_string(20));

    // Start a thread to manage rotation cycles
    auto rotationThread = std::thread([&]() {
        while (ros::ok() && isActive) {  // Run continuously while node is active
            // Calculate two random angular velocities
            double angularVelocity1 = calculateAngularVelocityZ(maxAngularVelocity); 
            double angularVelocity2 = calculateAngularVelocityZ(maxAngularVelocity); 
            logToFile(std::to_string(angularVelocity1));
            logToFile(std::to_string(angularVelocity2));
            
            double angularVelocities[4] = {
                angularVelocity1,  
                0,                
                angularVelocity2,  
                0                 
            };

            // Iterate through the angular velocity sequence
            for (int i = 0; i < 4; ++i) {
                twist.angular.z = angularVelocities[i];
                velPub.publish(twist);  
                if (verboseMode) {
                    ROS_INFO_STREAM("[" << nodeName << "] Setting angular velocity to: " << twist.angular.z);
                }

                ros::Time startTime = ros::Time::now(); 
                
                while ((ros::Time::now() - startTime) < ros::Duration(rotationDuration)) {
                    ros::spinOnce();  
                    ros::Duration(0.1).sleep();  

                    if (!isActive || !ros::ok()) {
                        cycleComplete = true;
                        twist.angular.z = 0;  
                        velPub.publish(twist); 
                        return; 
                    }
                }
            }
        }
        cycleComplete = true;
    });

    while (!cycleComplete && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    rotationThread.join();
    
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ [END ROTATION BASE SHIFT]------------------------------", nodeName.c_str());
    }

    logToFile("[END ROTATION BASE SHIFT]");
}

/**
 * @brief Executes flexible movements for the robot's hands and wrists.
 * 
 * This function controls the left and right arms and hands for the robot platform,
 * and only the arms for the simulator platform. It launches threads for simultaneous
 * control of the limbs based on the specified movement type.
 * 
 * @param nh The ROS NodeHandle to manage communication with ROS.
 * @param movementType A string indicating the type of movement ("flexi" or "All").
 */
void flexiMovement(ros::NodeHandle& nh, const std::string& movementType) {
    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------[START FLEXI MOVEMENT]------------------------------",nodeName.c_str());
    }
    logToFile("[START FLEXI MOVEMENT]");

    std::string rightArmTopic = topicData["RArm"];
    std::string leftArmTopic = topicData["LArm"];
    std::string rightHandTopic = topicData["RHand"];
    std::string leftHandTopic = topicData["LHand"];
    bool resetPosition = isFirstRun();

    if (resetPosition) {
        updateFirstRunFlag();
    }

    // Handle physical robot platform: Control arms, and hands simultaneously
    if (configParams["platform"] == "robot") {
        if (movementType == "flexi") {

            // Launch threads for controlling the wrist and hand of both left and right sides
            std::thread lArmlThread(lArml, std::ref(nh), leftArmTopic, resetPosition);
            std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
            std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
            std::thread rArmlThread(rArml, std::ref(nh), rightArmTopic, resetPosition);
            
            lArmlThread.join();
            lHandThread.join();
            rHandThread.join();
            rArmlThread.join();

        } else if (movementType == "All") {
            // Launch threads for controlling the wrist and hand of both left and right sides
            std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
            std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
            
            lHandThread.join();
            rHandThread.join();
        }
        logToFile("[END FLEXI MOVEMENT]");
    }

    // Handle simulator platform: Control arms only
    else if (configParams["platform"] == "simulator") {
        if (movementType == "flexi") {
            // Launch threads for controlling the wrist and hand of both left and right sides
            std::thread lArmlThread(lArml, std::ref(nh), leftArmTopic, resetPosition);
            std::thread rArmlThread(rArml, std::ref(nh), rightArmTopic, resetPosition);
          
            lArmlThread.join();
            rArmlThread.join();

        } else if (movementType == "All") {
            ROS_INFO("[%s]:flexiMovement: FlexiMovement function not used for body movement in simulator", nodeName.c_str());
            if(verboseMode) {
                ROS_INFO("[%s]:flexiMovement: FlexiMovement function not used for body movement in simulator", nodeName.c_str());
            }
        }
    }

    if (verboseMode) {
        ROS_INFO("[%s]:------------------------------ [END FLEXI MOVEMENT] ------------------------------", nodeName.c_str());
    }
    logToFile("[END FLEXI MOVEMENT]");
}

/**
* @brief Executes subtle body movements for the entire robot.
* 
* This function controls and coordinates movements of multiple robot parts 
* including arms, hands, and leg joints to create natural-looking animations.
* For the physical robot, it controls left/right arms, hands, and leg movements.
* For the simulator, it controls only the arms and leg movements (no hands).
* All movements are executed simultaneously using separate threads.
*
* @param nh The ROS NodeHandle to manage communication with ROS.
*/
void subtleBodyMovement(ros::NodeHandle& nh) {
    if (verboseMode) {
        ROS_INFO("[%s]: ------------------------------[START SUBTLE BODY MOVEMENT] ------------------------------", nodeName.c_str());
    }
    logToFile("[START SUBTLE BODY MOVEMENT]");
    
    std::string rightArmTopic = topicData["RArm"];
    std::string leftArmTopic = topicData["LArm"];
    std::string rightHandTopic = topicData["RHand"];
    std::string leftHandTopic = topicData["LHand"];
    std::string legTopic = topicData["Leg"];

    bool resetPosition = isFirstRun();
    if (resetPosition) {
        updateFirstRunFlag();
    }
    
    if (configParams["platform"] == "robot") {
            
            std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, resetPosition);
            std::thread lHandThread(lHand, std::ref(nh), leftHandTopic, resetPosition);
            std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, resetPosition);
            std::thread rHandThread(rHand, std::ref(nh), rightHandTopic, resetPosition);
            std::thread legThread(leg, std::ref(nh), legTopic, resetPosition);
                
            lArmThread.join();
            lHandThread.join();
            rArmThread.join();
            rHandThread.join();
            legThread.join(); 

    } else if (configParams["platform"] == "simulator") {

            std::thread lArmThread(lArm, std::ref(nh), leftArmTopic, resetPosition);
            std::thread rArmThread(rArm, std::ref(nh), rightArmTopic, resetPosition);
            std::thread legThread(leg, std::ref(nh), legTopic, resetPosition);
                
            lArmThread.join();
            rArmThread.join();
            legThread.join(); 

    }  

    if (verboseMode) {
        ROS_INFO("[%s]: ------------------------------ [END SUBTLE BODY MOVEMENT] --------------------", nodeName.c_str());
    }
    logToFile("[END SUBTLE BODY MOVEMENT]");         
}

/**
 * @brief Executes animation behaviors based on the given behavior parameter.
 * 
 * This function activates subtle body movements, hand flex movements, or base rotation
 * depending on the input. If no specific behavior is requested, it performs all actions.
 * 
 * @param behaviour A string indicating the desired animation behavior(s).
 * @param nh The ROS NodeHandle to manage communication with ROS.
 */
void animateBehaviour(const std::string& behaviour, ros::NodeHandle& nh) {
    
    /* Retrieve the platform from the configuration */
    std::string platform;
    try {
        platform = configParams.at("platform");  /* Use 'at' to catch out-of-range errors if the key doesn't exist */
    } catch (const std::out_of_range& e) {
        ROS_ERROR("[%s]:Configuration error: 'platform' not found in config file.", nodeName.c_str());
    }


    /* Validate the behaviour type: only 'body', 'hands', 'rotation', or an empty string should be accepted */
    std::vector<std::string> validBehaviours = {"body", "hands", "rotation", "All"};
    if (std::find(validBehaviours.begin(), validBehaviours.end(), behaviour) == validBehaviours.end()) {
        ROS_ERROR("[%s]:Invalid behaviour type: '%s'. Please choose one of the following: body, hands, rotation, or All.", nodeName.c_str(),behaviour.c_str());
    }

    /* Execute individual behaviors if specified */
    if (behaviour.find("body") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[%s]:Executing subtle body movements.", nodeName.c_str());
        }
        subtleBodyMovement(nh);
    }
    if (behaviour.find("hands") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[%s]: Executing hand flex movements.", nodeName.c_str());
        }
        flexiMovement(nh, "flexi");
    }
    if (behaviour.find("rotation") != std::string::npos) {
        if (verboseMode) {
            ROS_INFO("[%s]: Executing rotation base shift.", nodeName.c_str());
        }
        rotationBaseShift(nh);
    }

    /* Handle "All" behaviour - execute all movements with proper sequencing */
    if (behaviour.find("All") != std::string::npos) {
        logToFile("[START ALL ANIMATION BEHAVIOURS]");
        if (verboseMode) {
            ROS_INFO("[%s]: Executing All behaviors.", nodeName.c_str());
        }

        try {
            // Launch body thread with continuous execution
            boost::thread bodyThread([&nh]() {
                while (ros::ok() && isActive) {
                    subtleBodyMovement(nh);
                    ros::Duration(0.1).sleep();
                }
            });
            
            // Launch rotation thread
            boost::thread rotationThread([&nh]() {
                rotationBaseShift(nh);
                ros::Duration(5.0).sleep();  
            });
            
            // Launch flexi thread with continuous execution
            boost::thread flexiThread([&nh]() {
                while (ros::ok() && isActive) {
                    flexiMovement(nh, "All");
                    ros::Duration(0.1).sleep();
                }
            });

            bodyThread.join();
            rotationThread.join();
            flexiThread.join();

        } catch (const std::exception& e) {
            ROS_ERROR("[%s]: Error during All behaviors execution: %s", nodeName.c_str(), e.what());
            logToFile("Error during All behaviors execution: " + std::string(e.what()));
            cleanupAndCancelGoals(nh);
            throw;
        }

        logToFile("[END ALL ANIMATION BEHAVIOURS]");
    }

}




