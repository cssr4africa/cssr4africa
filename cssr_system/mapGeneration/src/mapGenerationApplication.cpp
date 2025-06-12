/* mapGenerationApplication.cpp
*
* This node is responsible for running the environment map generation node.
* The node is responsible for creation of empty maps, maps with obstacles, and configuration
* space generation with robot radius value.
*
* Libraries
* Standard libraries - std::string, std::vector, std::fstream, std::chrono, std::thread
* ROS libraries - ros/ros.h, ros/package.h
* OpenCV libraries - opencv2/opencv.hpp, opencv2/highgui.hpp, opencv2/imgproc.hpp
*
* Parameters
*
* Command-line Parameters
*
* None
*
* Configuration File Parameters
* Key                  | Value
* --------------------|-------------------
* mode                | CAD
* verboseMode         | true/false
* resolution          | 0.01
* robotRadius         | 0.3
* inputFile           | mapGenerationInput.dat
*
* Subscribed Topics and Message Types
*
* None
*
* Published Topics and Message Types
*
* None
*
* Services Invoked
*
* None
*
* Services Advertised and Message Types
*
* None
*
* Input Data Files
*
* mapGenerationInput.dat - Contains map dimensions and filenames
* obstacles.dat - Contains obstacle definitions
*
* Output Data Files
*
* environmentMap.png - Generated workspace map
* configurationSpaceMap.png - Generated configuration space map
*
* Configuration Files
*
* mapGenerationConfiguration.ini
*
* Example Instantiation of the Module
*
* rosrun cssr_system mapGeneration
*
* Author: Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email: bgirmash@andrew.cmu.edu
* Date: June 05, 2025
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network
* (Afretec) Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#include <mapGeneration/mapGenerationInterface.h>


int main(int argc, char **argv) {
  
   bool                 debug = true;
   
   FILE                 *fpIn = NULL;                    
   std::string          packageDir;
   char                 path[MAX_FILENAME_LENGTH];
   char                 inputFilename[MAX_FILENAME_LENGTH]                 = "mapGenerationInput.dat";
   char                 obstacleListFilename[MAX_FILENAME_LENGTH]          = "";
   char                 environmentMapFilename[MAX_FILENAME_LENGTH]        = "";
   char                 navigationMapFilename[MAX_FILENAME_LENGTH]         = "";
   char                 pathAndInputFilename[MAX_FILENAME_LENGTH]          = "";
   char                 configFilename[MAX_FILENAME_LENGTH]                = "";
   int                  endOfFile;
   bool                 success = true;
   configurationDataType configData;

   float                  xMapSize;
   float                  yMapSize;
   float                robotRadius;
   float                robotClearance = 0.05;          
   int                  i, j;
   int                  structuringElementSize;
   float                pixelsPerMeter;
   int                  xMapSizePixels;
   int                  yMapSizePixels;
   
   // Hardcoded configuration values
   float                imageDisplayScaleFactor = 0.4; // Scale the map images by this factor before displaying
   int                  compressionLevel = 9;           // PNG compression level (0=none, 9=max)

   vector<int> compressionParams;  // parameters for image write
   
   string mapWindowName                = "environmentMap.png"; 
   string configurationSpaceWindowName = "configurationSpaceMap.png";
   
   Mat mapImage;
   Mat mapImageColor;
   Mat mapImageSave;
   Mat configurationSpaceImage;
   Mat configurationSpaceImageColor;
   Mat configurationSpaceImageSave;
   Mat mapImageLarge;
   Mat configurationSpaceImageLarge;

   /* Set up ROS */
   /* ========== */
   
   /* Initialize the ROS system and become a node */
   /* ------------------------------------------- */
   
   ros::init(argc, argv, "mapGeneration"); // Initialize the ROS system
   ros::NodeHandle nh;                     // Become a node
   

   // Set up signal handler for clean shutdown
   signal(SIGINT, signalHandler);
   signal(SIGTERM, signalHandler);
   
   // Get the name of the node
   node_name = ros::this_node::getName();
   if (!node_name.empty() && node_name[0] == '/') {
      node_name = node_name.substr(1); // Remove the leading slash
   }

   // Print startup message
   formatStartupMessage();

   /* Read the configuration file */
   /* ========================== */
   try {
      packageDir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
      if (debug) ROS_INFO("mapGeneration: Package directory: %s", packageDir.c_str());
      
      strcpy(configFilename, packageDir.c_str());  
      strcat(configFilename, "/mapGeneration/config/mapGenerationConfiguration.ini");
      readConfigurationData(configFilename, &configData);
      
      // Use the configuration parameters
      debug = configData.verboseMode;
      
      /* Check operating mode */
      if (strcmp(configData.mode, "SLAM") == 0) {
         ROS_INFO("SLAM mode is not yet implemented");
         promptAndExit(1);
      }

      pathAndInputFilename[0] = '\0';  // Clear the string
      strcpy(pathAndInputFilename, packageDir.c_str());  
      strcat(pathAndInputFilename, "/mapGeneration/data/"); 
      strcat(pathAndInputFilename, configData.inputFile);
      
      if (debug) ROS_INFO("mapGeneration: Input file is %s", pathAndInputFilename);
      
      /* Auto-detect file format and read accordingly */
      /* -------------------------------------------- */
      if (isKeyValueFormat(pathAndInputFilename)) {
         if (debug) ROS_INFO("mapGeneration: Detected key-value format input file");
         readInputDataKeyValue(pathAndInputFilename, &xMapSize, &yMapSize, 
                              obstacleListFilename,
                              environmentMapFilename, navigationMapFilename);
        

      } else {
         if (debug) ROS_INFO("mapGeneration: Detected legacy positional format input file");
         
         if ((fpIn = fopen(pathAndInputFilename,"r")) == 0) {
            ROS_ERROR("mapGeneration: Error: can't open %s", pathAndInputFilename);
            promptAndExit(1);
         }
         
         /* get the dimensions of the map in centimeters */
         /* -------------------------------------------- */
         endOfFile = fscanf(fpIn, "%f %f", &xMapSize, &yMapSize);  // Read as floats         
         if (endOfFile == EOF) {   
            ROS_ERROR("mapGeneration: Fatal error: unable to read the dimensions of the map");
            fclose(fpIn);
            promptAndExit(1);
         }
      
         /* get the obstacle list filename  */
         /* ------------------------------- */
         endOfFile = fscanf(fpIn, "%s", obstacleListFilename);
         if (endOfFile == EOF) {   
            ROS_ERROR("mapGeneration: Fatal error: unable to read the obstacle list filename");
            fclose(fpIn);
            promptAndExit(1);
         }
      
         /* get the environment map filename  */
         /* --------------------------------- */
         endOfFile = fscanf(fpIn, "%s", environmentMapFilename);
         if (endOfFile == EOF) {   
            ROS_ERROR("mapGeneration: Fatal error: unable to read the environment map filename");
            fclose(fpIn);
            promptAndExit(1);
         }
      
         /* get the navigation map filename  */
         /* ------------------------------- */
         endOfFile = fscanf(fpIn, "%s", navigationMapFilename);
         if (endOfFile == EOF) {   
            ROS_ERROR("mapGeneration: Fatal error: unable to read the navigation map filename");
            fclose(fpIn);
            promptAndExit(1);
         }
      
         // Close the input file after reading all data
         fclose(fpIn);
         fpIn = NULL;
      }

      float pixelsPerMeter = 1.0 / configData.resolution;  // resolution is meters per pixel
      int xMapSizePixels = (int)(xMapSize * pixelsPerMeter);
      int yMapSizePixels = (int)(yMapSize * pixelsPerMeter);
      
      /* set robot radius from config file */
      /* --------------------------------- */
      robotRadius = configData.robotRadius;  // Use config file value only
      
      if (debug) {
         ROS_INFO("mapGeneration: Using robot radius: %f", robotRadius);
      }

      if (debug) {
         ROS_INFO("mapGeneration: Dimensions of the map %.2f x %.2f m (%d x %d pixels)", xMapSize, yMapSize, xMapSizePixels, yMapSizePixels);
         ROS_INFO("mapGeneration: Obstacle list filename %s", obstacleListFilename);
         ROS_INFO("mapGeneration: Robot radius  %5.3f", robotRadius);
         ROS_INFO("mapGeneration: Environment map filename %s", environmentMapFilename);
         ROS_INFO("mapGeneration: Configuration space map filename %s", navigationMapFilename);
      }

      // Start heartbeat thread
      std::thread heartbeatThread([]() {
         while (ros::ok() && !shutdown_requested) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            if (!shutdown_requested) {
               printHeartbeat();
            }
         }
      });
      heartbeatThread.detach();

      /* Initialize the map and configuration space */
      /* ========================================== */
      
      /* create the map and configuration space images */
      /* --------------------------------------------- */
      mapImage.create(yMapSizePixels, xMapSizePixels, CV_8UC1);
      configurationSpaceImage.create(yMapSizePixels, xMapSizePixels, CV_8UC1);

      /* read the obstacle list data and initialize the map */
      /* -------------------------------------------------- */
      pathAndInputFilename[0] = '\0';  // Clear the string
      strcpy(pathAndInputFilename, packageDir.c_str());  
      strcat(pathAndInputFilename, "/mapGeneration/data/"); 
      strcat(pathAndInputFilename, obstacleListFilename);
      
      readObstacleData(pathAndInputFilename, mapImage, configData.resolution);  

      /* add a wall around the map, i.e. a one-pixel boundary so that the robot cannot navigate outside the known environment */

      for (i = 0; i < mapImage.rows; i++) {
         for (j = 0; j < mapImage.cols; j++) {
            if (i == 0 || i == mapImage.rows - 1 || j == 0 || j == mapImage.cols - 1)
               mapImage.at<uchar>(i, j) = (uchar)0;
         }
      }

      /* generate the configuration space */
      /* -------------------------------- */

      configurationSpaceImage = mapImage.clone();

    
      /* create a structuring element the size of the robot to dilate the map and generate the configuration space */
      
      structuringElementSize = 2 * (int) ((robotRadius + robotClearance) * pixelsPerMeter) + 1;   

      Mat structuringElement(structuringElementSize, structuringElementSize, CV_8U, Scalar(1));  

      for (i = 0; i < structuringElement.rows; i++) {
         for (j = 0; j < structuringElement.cols; j++) {
            if (sqrt((i - structuringElementSize / 2) * (i - structuringElementSize / 2) + 
               (j - structuringElementSize / 2) * (j - structuringElementSize / 2)) <= 
               (int)((robotRadius + robotClearance) * pixelsPerMeter)) {
               structuringElement.at<uchar>(i, j) = (uchar)1;
            } else {
               structuringElement.at<uchar>(i, j) = (uchar)0;
            }
         }
      }

      erode(mapImage, configurationSpaceImage, structuringElement,
            Point((int) ((robotRadius + robotClearance) * pixelsPerMeter), 
                  (int) ((robotRadius + robotClearance) * pixelsPerMeter)), 1);
      
      // Add axes to both maps
      Mat mapImageWithAxes = mapImage.clone();
      Mat configurationSpaceImageWithAxes = configurationSpaceImage.clone();
      drawAxes(mapImageWithAxes);
      drawAxes(configurationSpaceImageWithAxes);

      // Set up compression parameters
      compressionParams.push_back(IMWRITE_PNG_COMPRESSION);
      compressionParams.push_back(compressionLevel);

      /* Save environment map */
      /* -------------------- */
      pathAndInputFilename[0] = '\0';
      strcpy(pathAndInputFilename, packageDir.c_str());  
      strcat(pathAndInputFilename, "/mapGeneration/data/"); 
      strcat(pathAndInputFilename, environmentMapFilename);

      if (!imwrite(pathAndInputFilename, mapImageWithAxes, compressionParams)) {
         ROS_ERROR("mapGeneration: Failed to save environment map to %s", pathAndInputFilename);
      } else {
         ROS_INFO("mapGeneration: Successfully saved environment map to %s", pathAndInputFilename);
      }

      /* Save configuration space map */
      /* --------------------------- */
      pathAndInputFilename[0] = '\0';
      strcpy(pathAndInputFilename, packageDir.c_str());  
      strcat(pathAndInputFilename, "/mapGeneration/data/");
      strcat(pathAndInputFilename, navigationMapFilename);  // Use filename from input file

      if (!imwrite(pathAndInputFilename, configurationSpaceImageWithAxes, compressionParams)) {
         ROS_ERROR("mapGeneration: Failed to save configuration space map to %s", pathAndInputFilename);
      } else {
         ROS_INFO("mapGeneration: Successfully saved configuration space map to %s", pathAndInputFilename);
      }

      /* Display the maps only if in verbose mode */
      /* --------------------------------------- */
      if (configData.verboseMode && !shutdown_requested) {
         ROS_INFO("mapGeneration: Maps have been saved to:");
         ROS_INFO("mapGeneration: Environment map: %s/mapGeneration/data/%s", packageDir.c_str(), environmentMapFilename);
         ROS_INFO("mapGeneration: Configuration space map: %s/mapGeneration/data/%s", packageDir.c_str(), navigationMapFilename);
         ROS_INFO("mapGeneration: Displaying maps...");
         
         // Convert to color for display
         cvtColor(mapImageWithAxes, mapImageColor, COLOR_GRAY2BGR);
         cvtColor(configurationSpaceImageWithAxes, configurationSpaceImageColor, COLOR_GRAY2BGR);
         
         // Scale the maps for display
         resize(mapImageColor, mapImageLarge, Size(), imageDisplayScaleFactor, imageDisplayScaleFactor);
         resize(configurationSpaceImageColor, configurationSpaceImageLarge, Size(), imageDisplayScaleFactor, imageDisplayScaleFactor);
         
         // Display the maps
         namedWindow(mapWindowName, WINDOW_AUTOSIZE);
         namedWindow(configurationSpaceWindowName, WINDOW_AUTOSIZE);
         imshow(mapWindowName, mapImageLarge);
         imshow(configurationSpaceWindowName, configurationSpaceImageLarge);
         
         ROS_INFO("mapGeneration: Press Ctrl+C to exit...");
         
         // Non-blocking wait loop that respects shutdown signals
         int key = 0;
         while (key != 27 && !shutdown_requested && ros::ok()) {  // 27 = ESC key
            key = cv::waitKey(30);  // 30ms timeout
            ros::spinOnce();  // Allow ROS to process callbacks and check for shutdown
            
            // Break on any key press (except for -1 which means timeout)
            if (key >= 0) {
               break;
            }
         }
         
         // Clean up OpenCV windows
         cv::destroyAllWindows();
            // Success message after display
         if (!shutdown_requested) {
            ROS_INFO("mapGeneration: Maps displayed successfully.");
         }
      }
      
      if (!shutdown_requested) {
         ROS_INFO("mapGeneration: Map generation completed successfully.");
      }
      promptAndExit(0);
   }
   catch (const exception& e) {
      // Handle any exceptions that occur during execution
      ROS_ERROR("mapGeneration: An exception occurred: %s", e.what());
      
      // Clean up resources
      if (fpIn != NULL) {
         fclose(fpIn);
      }
      
      // Only destroy windows if they were created
      if (configData.verboseMode) {
         cv::destroyAllWindows();
      }
      
      return 1;
   }
      
   return 0;
}