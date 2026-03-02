/* mapGenerationImplementation.cpp
*
* Author: Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email: bgirmash@andrew.cmu.edu
* Date: June 05, 2025
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

#include <mapGeneration/mapGenerationInterface.h> 

// Define the global variable
std::string node_name;
// Define the global shutdown flag
volatile sig_atomic_t shutdown_requested = 0;
/*******************************************************************************
   formatStartupMessage
   
   Print formatted startup message with copyright notice
*******************************************************************************/
void formatStartupMessage() {
    // Copyright message
   ROS_INFO("%s %s", node_name.c_str(), SOFTWARE_VERSION);
   printf("\n");
   printf("\t\t\t\tThis project is funded by the African Engineering and Technology Network\n");
   printf("\t\t\t\t(Afretec) Inclusive Digital Transformation Research Grant Programme.\n");
   printf("\t\t\t\tWebsite: www.cssr4africa.org\n");
   printf("\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY\n");
    
    // Startup message
   printf("\n");
   ROS_INFO("%s: start-up.", node_name.c_str());
}

/*******************************************************************************
   printHeartbeat
   
   Print periodic heartbeat message
*******************************************************************************/
void printHeartbeat() {
    ROS_INFO("%s: running.", node_name.c_str());
}

/*******************************************************************************
   drawAxes
   
   Draw coordinate axes on the map
*******************************************************************************/
void drawAxes(Mat& map) {
    // Convert to color image if grayscale
    Mat colorMap;
    if (map.channels() == 1) {
        cvtColor(map, colorMap, COLOR_GRAY2BGR);
    } else {
        colorMap = map.clone();
    }
    
    // Draw axes at bottom-left corner
    int margin = 50;
    cv::Point origin(margin, colorMap.rows - margin);
    cv::Point xEnd(margin + 100, colorMap.rows - margin);
    cv::Point yEnd(margin, colorMap.rows - margin - 100);
    
    // X-axis (red) - pointing right
    cv::arrowedLine(colorMap, origin, xEnd, cv::Scalar(0, 0, 255), 2);
    cv::putText(colorMap, "X (forward)", xEnd + cv::Point(10, 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    
    // Y-axis (green) - pointing up (left in robot frame)
    cv::arrowedLine(colorMap, origin, yEnd, cv::Scalar(0, 255, 0), 2);
    cv::putText(colorMap, "Y (left)", yEnd + cv::Point(-20, -10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    
    // Add origin label
    cv::putText(colorMap, "Origin", origin + cv::Point(5, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // Convert back to grayscale if needed
    if (map.channels() == 1) {
        cvtColor(colorMap, map, COLOR_BGR2GRAY);
    } else {
        map = colorMap;
    }
}

/*******************************************************************************
   readConfigurationData
   
   Read configuration data from file     
*******************************************************************************/
void readConfigurationData(char filename[], configurationDataType *configData)
{
   bool debug = false;
   FILE *fp_config;
   char inputString[STRING_LENGTH];
   char key[KEY_LENGTH];
   char value[STRING_LENGTH];

   // Set default values
   strcpy(configData->mode, "CAD");
   configData->verboseMode = false;
   strcpy(configData->inputFile, "mapGenerationInput.dat");
   configData->resolution = 0.05;
   configData->robotRadius = 0.3;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
     ROS_ERROR("%s: Error can't open configuration file %s", node_name.c_str(), filename);
     promptAndExit(0);
   }

   // Read each line and extract key-value pairs
   while (fgets(inputString, STRING_LENGTH, fp_config) != NULL)
   {
      // Skip empty lines and comments
      if (inputString[0] == '\n' || inputString[0] == '#')
         continue;
      
      sscanf(inputString, " %s %s", key, value);
      
      // Check which key it is
      if (strcmp(key, "mode") == 0) {
         strcpy(configData->mode, value);
      }
      else if (strcmp(key, "verboseMode") == 0) {
         if (strcmp(value, "true") == 0)
            configData->verboseMode = true;
         else
            configData->verboseMode = false;
      }
      else if (strcmp(key, "inputFile") == 0) {
         strcpy(configData->inputFile, value);
      }
      else if (strcmp(key, "resolution") == 0) {
         configData->resolution = atof(value);
      }
      else if (strcmp(key, "robotRadius") == 0) {
         configData->robotRadius = atof(value);
      }
   }

   fclose(fp_config);

   if (debug)
   {
     ROS_INFO("%s: Configuration parameters:", node_name.c_str());
     ROS_INFO("  mode: %s", configData->mode);
     ROS_INFO("  verboseMode: %s", configData->verboseMode ? "true" : "false");
     ROS_INFO("  inputFile: %s", configData->inputFile);
     ROS_INFO("  resolution: %f", configData->resolution);
     ROS_INFO("  robotRadius: %f", configData->robotRadius);
   }
}


/*******************************************************************************
   isKeyValueFormat
   
   Check if input file is in key-value format by looking at first line
*******************************************************************************/
bool isKeyValueFormat(char filename[]) {
   FILE *fp;
   char firstLine[STRING_LENGTH];
   char firstWord[KEY_LENGTH];
   
   if ((fp = fopen(filename, "r")) == 0) {
      return false;  // If can't open, assume old format
   }
   
   // Read first non-comment line
   while (fgets(firstLine, STRING_LENGTH, fp) != NULL) {
      if (firstLine[0] != '\n' && firstLine[0] != '#' && firstLine[0] != '\0') {
         sscanf(firstLine, " %s", firstWord);
         fclose(fp);
         
         // Check if first word looks like a key (contains letters)
         // Old format starts with numbers (680), new format starts with "mapWidth"
         return !isdigit(firstWord[0]);
      }
   }
   
   fclose(fp);
   return false;  // Default to old format
}

/*******************************************************************************
   readInputDataKeyValue
   
   Read input data from file using key-value pairs
*******************************************************************************/
void readInputDataKeyValue(char filename[], float *xMapSize, float *yMapSize, 
                          char obstacleListFilename[],
                          char environmentMapFilename[], char navigationMapFilename[]) {
   
   bool debug = false;
   FILE *fp_input;
   char inputString[STRING_LENGTH];
   char key[KEY_LENGTH];
   char value[STRING_LENGTH];
   
   // Flags to check required parameters
   bool mapWidthFound = false, mapHeightFound = false;
   bool obstacleFileFound = false, envMapFileFound = false, configMapFileFound = false;
   
  
   
   if ((fp_input = fopen(filename, "r")) == 0) {
      ROS_ERROR("%s: Error can't open input file %s", node_name.c_str(), filename);
      promptAndExit(1);
   }
   
   // Read each line and extract key-value pairs
   while (fgets(inputString, STRING_LENGTH, fp_input) != NULL) {
      // Skip empty lines and comments
      if (inputString[0] == '\n' || inputString[0] == '#' || inputString[0] == '\0')
         continue;
      
      if (sscanf(inputString, " %s %s", key, value) != 2)
         continue;  // Skip malformed lines
      
      // Process each key
      if (strcmp(key, "mapWidth") == 0) {
         *xMapSize = atof(value);  // Use atof instead of atoi
         mapWidthFound = true;
         if (debug) ROS_INFO("Map width: %.2f m", *xMapSize);
      }
      else if (strcmp(key, "mapHeight") == 0) {
         *yMapSize = atof(value);  // Use atof instead of atoi  
         mapHeightFound = true;
         if (debug) ROS_INFO("Map height: %.2f m", *yMapSize);
      }
      else if (strcmp(key, "obstacleFile") == 0) {
         strcpy(obstacleListFilename, value);
         obstacleFileFound = true;
         if (debug) ROS_INFO("Obstacle file: %s", obstacleListFilename);
      }
      else if (strcmp(key, "environmentMapFile") == 0) {
         strcpy(environmentMapFilename, value);
         envMapFileFound = true;
         if (debug) ROS_INFO("Environment map file: %s", environmentMapFilename);
      }
      else if (strcmp(key, "configurationSpaceMapFile") == 0) {
         strcpy(navigationMapFilename, value);
         configMapFileFound = true;
         if (debug) ROS_INFO("Configuration space map file: %s", navigationMapFilename);
      }
      else {
         ROS_WARN("Unknown key in input file: %s", key);
      }
   }
   
   fclose(fp_input);
   
   // Validate required parameters
   if (!mapWidthFound) {
      ROS_ERROR("Fatal error: mapWidth not found in input file");
      promptAndExit(1);
   }
   if (!mapHeightFound) {
      ROS_ERROR("Fatal error: mapHeight not found in input file");
      promptAndExit(1);
   }
   if (!obstacleFileFound) {
      ROS_ERROR("Fatal error: obstacleFile not found in input file");
      promptAndExit(1);
   }
   if (!envMapFileFound) {
      ROS_ERROR("Fatal error: environmentMapFile not found in input file");
      promptAndExit(1);
   }
   if (!configMapFileFound) {
      ROS_ERROR("Fatal error: configurationSpaceMapFile not found in input file");
      promptAndExit(1);
   }
}
/*******************************************************************************
   readObstacleData
   
   Read obstacle data from file     
*******************************************************************************/
void readObstacleData(char filename[], Mat &map, float resolution){
   bool debug = false;
   int i;
   int j;
   float xCentroid;
   float yCentroid;
   float xDim;
   float yDim;
   float radius;

   keyword keylist[3] = {
       "obstacle",
       "rectangle",
       "circle"
   };

   keyword key;
   char inputString[STRING_LENGTH];
   FILE *fp_config;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
      ROS_ERROR("%s: Error can't open obstacle list file %s", node_name.c_str(), filename);
      promptAndExit(0);
   }

   // Initialize the map (white background)
   for (i = 0; i < map.rows; i++)
   {
      for (j = 0; j < map.cols; j++)
      {
         map.at<uchar>(i, j) = (uchar)255;
      }
   }

   // Get the key-value pairs
   while (fgets(inputString, STRING_LENGTH, fp_config) != NULL)
   {
      if (debug)
         ROS_INFO("Input string: %s", inputString);

      // Extract the key
      sscanf(inputString, " %s", key);

      for (j = 0; j < (int)strlen(key); j++)
         key[j] = tolower(key[j]);

      if (debug)
         ROS_INFO("key: %s", key);

      // Handle both "OBSTACLE" and "RECTANGLE" as rectangular obstacles
      if (strcmp(key, "obstacle") == 0 || strcmp(key, "rectangle") == 0)
      {
         sscanf(inputString, " %s %f %f %f %f", key, &xCentroid, &yCentroid, &xDim, &yDim);
         if (debug)
            ROS_INFO("OBSTACLE: %5.3f, %5.3f, %5.3f, %5.3f", xCentroid, yCentroid, xDim, yDim);

         // Convert to pixels using resolution (obstacles are already in meters)
         float pixelsPerMeter = 1.0 / resolution;  // Use resolution from config
         xCentroid *= pixelsPerMeter;
         yCentroid *= pixelsPerMeter;
         xDim *= pixelsPerMeter;
         yDim *= pixelsPerMeter;

         for (i = 0; i < map.rows; i++)
         {
            for (j = 0; j < map.cols; j++)
            {
               if (((map.rows - i) >= (yCentroid - yDim / 2)) && 
                   ((map.rows - i) <= (yCentroid + yDim / 2)) && 
                   (j >= (xCentroid - xDim / 2)) && 
                   (j <= (xCentroid + xDim / 2)))
               {
                  map.at<uchar>(i, j) = (uchar)0;
               }
            }
         }
      }
      else if (strcmp(key, "circle") == 0)
      {
         sscanf(inputString, " %s %f %f %f", key, &xCentroid, &yCentroid, &radius);
         if (debug)
            ROS_INFO("CIRCLE: %5.3f, %5.3f, %5.3f", xCentroid, yCentroid, radius);

         // Convert to pixels using resolution 
         float pixelsPerMeter = 1.0 / resolution;   // Use resolution from config
         xCentroid *= pixelsPerMeter;
         yCentroid *= pixelsPerMeter;
         radius *= pixelsPerMeter;

         for (i = 0; i < map.rows; i++)
         {
            for (j = 0; j < map.cols; j++)
            {
               if (sqrt(((map.rows - i) - yCentroid) * ((map.rows - i) - yCentroid) + 
                        (j - xCentroid) * (j - xCentroid)) <= radius)
               {
                  map.at<uchar>(i, j) = (uchar)0;
               }
            }
         }
      }
   }
   
   fclose(fp_config);
}

/***************************************************************************************************************************
   Function definitions for the mapping between graph and map data structures
****************************************************************************************************************************/

int rowNumber(int vertexNumber, int numberOfColumns)
{
   int n;
   n = (vertexNumber - 1) / numberOfColumns;
   return (n);
}

int columnNumber(int vertexNumber, int numberOfColumns)
{
   int n;
   n = (vertexNumber - 1) % numberOfColumns;
   return (n);
}

int vertexNumber(int row, int column, int numberOfColumns)
{
   int n;
   n = (row * numberOfColumns) + column + 1;
   return (n);
}


/*******************************************************************************
   signalHandler
   
   Handle SIGINT and SIGTERM signals for clean shutdown
*******************************************************************************/
void signalHandler(int sig) {
   shutdown_requested = 1;
   cv::destroyAllWindows();  // Close all OpenCV windows immediately
   ros::shutdown();
}

/***************************************************************************************************************************
   General purpose function definitions 
****************************************************************************************************************************/

/* return the sign of a number as +/- 1 */
int signnum(float x)
{
   if (x >= 0.0)
     return 1;
   return -1;
}

void displayErrorAndExit(char error_message[])
{
   ROS_ERROR("%s: %s", node_name.c_str(), error_message);
   promptAndExit(1);
}

void promptAndExit(int status)
{
   ROS_INFO("%s: Exiting...", node_name.c_str());
   ros::shutdown();
   exit(status);
}

void promptAndContinue()
{
   // printf("Press any key to continue ... \n");
   getchar();
}

void printMessageToFile(FILE *fp, char message[])
{
   fprintf(fp, "The message is: %s\n", message);
}

#ifdef ROS
/**
 * Linux (POSIX) implementation of _kbhit().
 * Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif