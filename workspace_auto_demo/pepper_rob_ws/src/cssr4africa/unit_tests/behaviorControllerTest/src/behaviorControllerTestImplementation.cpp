/* behaviorControllerTestImplementation.cpp   Source code for the methods used by behaviiorControllerTestApplication
 *
 * Author: Tsegazeab Taye Tefferi
 * Date: April 25, 2025
 * Version: 1.0
 *
 * Copyright (C) 2023 CSSR4Africa Consortium
 *
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 *
 * Website: www.cssr4africa.org
 *
 * This program comes with ABSOLUTELY NO WARRANTY.
 *
 */

#include <behaviorControllerTest/behaviorControllerTestInterface.h>

float failureRate;

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
template<typename T>
void printMsg(int type,const T& args){
    if(!verboseMode){
        return;
    }

    std::stringstream msg;
    msg << "behaviorControllerTest: " << args;
    switch (type){
        case INFO_MSG:
            ROS_INFO_STREAM(msg.str());
            break;
        case WARNING_MSG:
            ROS_WARN_STREAM(msg.str());
            break;
        case ERROR_MSG:
            ROS_ERROR_STREAM(msg.str());
            break;
        default:
            ROS_ERROR_STREAM("UNDEFINED MSG TYPE");
    }
}

/* Returns true if failureRate is lessthan a random evaluated number */
bool hasSucceeded() {
    return (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) > failureRate;
}

/* Returns true if ch isn't an empty space character*/
static bool isNotSpace(unsigned char ch) {
    return !std::isspace(ch);
}

/* Trims whitespaces inplace */
static inline void trim(std::string &s) {
    // Trim leading spaces
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), isNotSpace));
    
    // Trim trailing spaces
    s.erase(std::find_if(s.rbegin(), s.rend(), isNotSpace).base(), s.end());
}

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key) {
    std::string value; 
    std::ifstream configFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorControllerTest/config/behaviorControllerTestConfiguration.ini");
    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    std::string configLineRead;

    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        
        trim(paramKey);
        std::getline(iss, paramValue);
        trim(paramValue);
        
        if (paramKey == key) { 
            value = paramValue;
            break;
        }
    }
    configFile.close();
    if (value.empty()) {
        throw std::runtime_error("Failed to retrieve value for key: " + key);
    }
    return value;
}


// At the end of your .cpp file where printMsg is defined
template void printMsg<std::string>(int, const std::string&);
template void printMsg<char[22]>(int, const char(&)[22]);
template void printMsg<char[49]>(int, const char(&)[49]);
template void printMsg<char[50]>(int, const char(&)[50]);
template void printMsg<char[24]>(int, const char(&)[24]);
template void printMsg<char[27]>(int, const char(&)[27]);
