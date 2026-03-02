/* mapGenerationTestInterface.h
*
* Author: Birhanu Shimelis Girma
* Date: April 08, 2025
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

#ifndef MAP_GENERATION_TEST_INTERFACE_H
#define MAP_GENERATION_TEST_INTERFACE_H

#include <gtest/gtest.h> 

#define ROS
 
#ifndef ROS
   #include <conio.h>
#else
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <sys/select.h>
   #include <termios.h>
   #include <sys/ioctl.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

/***************************************************************************************************************************
   ROS package name
****************************************************************************************************************************/
#define ROS_PACKAGE_NAME    "unit_tests"
// Software version
#define SOFTWARE_VERSION                    "v1.0"

/***************************************************************************************************************************
   General purpose definitions 
****************************************************************************************************************************/
#define PI       3.14159
#define TRUE     1
#define FALSE    0

/***************************************************************************************************************************
   File I/O and data structure definitions 
****************************************************************************************************************************/
#define MAX_FILENAME_LENGTH 200
#define STRING_LENGTH       200
#define KEY_LENGTH           40

typedef char keyword[KEY_LENGTH];

/***************************************************************************************************************************
   Definitions for the configuration data structure (for test configuration only)
***************************************************************************************************************************/
typedef struct {
   char mode[20];           // "CAD" or "SLAM"
   bool verboseMode;        // true or false
   char inputFile[200];     // path to input file
   float resolution;        // map resolution
   float robotRadius;       // robot radius for configuration space
} configurationDataType;

extern std::string node_name;   
extern std::ofstream logFile;  // Declare the global log file   
extern std::ofstream logFile;  // Declare the global log file

/***************************************************************************************************************************
   Test helper function declarations - these functions help test the actual mapGeneration node
****************************************************************************************************************************/

namespace mapGenerationTest {

// Forward declarations
class FormattedOutputListener;
class MapGenerationTest;

// Logging functions
void LogTestStep(const std::string& message);
void LogTestSuccess(const std::string& message);
void LogTestPassed(const std::string& message);

// Custom test event listener class declaration
class FormattedOutputListener : public ::testing::EmptyTestEventListener {
private:
    std::string getCurrentTimeString();

public:
    void OnTestProgramStart(const ::testing::UnitTest& unit_test) override;
    void OnTestStart(const ::testing::TestInfo& test_info) override;
    void OnTestEnd(const ::testing::TestInfo& test_info) override;
    void OnTestProgramEnd(const ::testing::UnitTest& unit_test) override;
};

// File and validation utilities
bool fileExists(const std::string& filepath);
long getFileSize(const std::string& filepath);
bool validateMapProperties(const std::string& mapPath, int expectedWidth, int expectedHeight);
bool compareConfigurationSpaces(const std::string& envMapPath, const std::string& configMapPath);

// Map analysis utilities
int countObstaclePixels(const cv::Mat& map);

// Input file utilities
bool getExpectedDimensions(const std::string& inputFilePath, int& width, int& height);
bool createTestInputFile(const std::string& filepath, 
                        int width, 
                        int height, 
                        const std::string& obstacleFile,
                        const std::string& envMapFile,
                        const std::string& configMapFile);

// Configuration file utilities
bool createTestConfigFile(const std::string& filepath, 
                         float robotRadius, 
                         bool verboseMode,
                         const std::string& inputFile);

// Node testing utilities
bool launchMapGenerationNode(const std::string& packagePath, int timeoutSeconds);
bool verifyNodeOutputs(const std::string& dataDir, 
                      const std::string& expectedEnvMap,
                      const std::string& expectedConfigMap,
                      int expectedWidth,
                      int expectedHeight);

// Test environment setup
bool setupTestEnvironment(const std::string& testDataDir, 
                         const std::string& mapGenDataDir,
                         const std::string& inputFileName,
                         const std::string& obstacleFileName);

bool copyFile(const std::string& source, const std::string& destination);
void cleanupTestFiles(const std::string& dataDir);

// Main test function - tests the actual mapGeneration node
bool testActualMapGenerationNode(const std::string& testName,
                                const std::string& inputFileName,
                                const std::string& configFileName,
                                float expectedRobotRadius,
                                int expectedWidth,
                                int expectedHeight);

/***************************************************************************************************************************
   General purpose function declarations 
****************************************************************************************************************************/
void display_error_and_exit(const char* error_message);
void prompt_and_exit(int status);
void prompt_and_continue();

#ifdef ROS
   int _kbhit();
#endif

} // namespace mapGenerationTest

#endif // MAP_GENERATION_TEST_INTERFACE_H