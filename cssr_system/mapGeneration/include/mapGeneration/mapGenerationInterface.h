/* mapGenerationInterface.h
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

#ifndef MAP_GENERATION_INTERFACE_H
#define MAP_GENERATION_INTERFACE_H

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
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

/***************************************************************************************************************************
   ROS package name
****************************************************************************************************************************/
#define ROS_PACKAGE_NAME    "cssr_system"
// Software version
#define SOFTWARE_VERSION                    "v1.0"
/***************************************************************************************************************************
   General purpose definitions 
****************************************************************************************************************************/
#define PI       3.14159
#define TRUE     1
#define FALSE    0
#define BFS      0
#define DIJKSTRA 1
#define ASTAR    2

/***************************************************************************************************************************
   File I/O and data structure definitions 
****************************************************************************************************************************/
#define MAX_FILENAME_LENGTH 200
#define STRING_LENGTH       200
#define KEY_LENGTH           40
#define NUMBER_OF_KEYS       14

typedef char keyword[KEY_LENGTH];


/***************************************************************************************************************************
   Definitions for the configuration data structure
***************************************************************************************************************************/

typedef struct {
   char mode[20];           // "CAD" or "SLAM"
   bool verboseMode;        // true or false
   char inputFile[200];     // path to input file
   float resolution;        // map resolution
   float robotRadius;       // robot radius for configuration space
} configurationDataType;

extern std::string node_name;   
extern volatile sig_atomic_t shutdown_requested;
void signalHandler(int sig);

/***************************************************************************************************************************
   Point type for map generation
****************************************************************************************************************************/
typedef struct  {
   int x;
   int y;
} pointType;

/***************************************************************************************************************************
   Function declarations for reading input data
****************************************************************************************************************************/
bool isKeyValueFormat(char filename[]);
void readInputDataKeyValue(char filename[], float *xMapSize, float *yMapSize, 
                          char obstacleListFilename[], char environmentMapFilename[], char navigationMapFilename[]);


/***************************************************************************************************************************
   Function declarations for map generation 
****************************************************************************************************************************/
void readConfigurationData(char filename[], configurationDataType *configData);
void readObstacleData(char filename[], Mat &map, float resolution);
void formatStartupMessage();
void printHeartbeat();
void drawAxes(Mat& map);

/***************************************************************************************************************************
   Signal handling functions
****************************************************************************************************************************/
void signalHandler(int sig);
void setupSignalHandlers();

/***************************************************************************************************************************
   Function declarations for mapping between coordinates
****************************************************************************************************************************/
int rowNumber(int vertexNumber, int numberOfColumns);
int columnNumber(int vertexNumber, int numberOfColumns);
int vertexNumber(int row, int column, int numberOfColumns);

/***************************************************************************************************************************
   General purpose function declarations 
****************************************************************************************************************************/
void displayErrorAndExit(char error_message[]);
void promptAndExit(int status);
void promptAndContinue();
void printMessageToFile(FILE *fp, char message[]);
int  signnum(float x); // return the sign of a number

#ifdef ROS
   int _kbhit();
#endif

#endif // MAP_GENERATION_INTERFACE_H