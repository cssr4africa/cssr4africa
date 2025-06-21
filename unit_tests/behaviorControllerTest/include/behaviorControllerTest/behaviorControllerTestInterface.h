/* behaviorControllerTestInterface.h - interface file for behaviorControllerTestApplication and behaivorControllerTestImplementation
*
* Author: Tsegazeab Taye Tefferi
* Date: April 20, 2025
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

#ifndef BEHAVIOR_CONTROLLER_TEST_INTERFACE_H
#define BEHAVIOR_CONTROLLER_TEST_INTERFACE_H

#include "ros/ros.h"
#include <fstream>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>


//Configuration variables
extern bool verboseMode;
extern std::string nodeName;
extern float failureRate;

/* Definitions for printMsg function */
#define INFO_MSG 0
#define WARNING_MSG 1
#define ERROR_MSG 2

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
template<typename T>
void printMsg(int type,const T& args);

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key);

/* Returns true if failureRate is lessthan a random evaluated number */
bool hasSucceeded();

#endif