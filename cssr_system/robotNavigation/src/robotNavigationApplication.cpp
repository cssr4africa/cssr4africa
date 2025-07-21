/* robotNavigationApplication.cpp - robot navigation ROS Node definition
 *
 * Author:  Birhanu Shimelis Girma, Carnegie Mellon University Africa
 * Email:   bgirmash@andrew.cmu.edu
 * Date:    June 05, 2025
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

/* robotNavigationApplication.cpp - Application code to run the robot Navigation.

 * This node is responsible for navigating the robot to a goal location in the
 * environment map using a path planning algorithm. The node reads the robot
 * pose input and the goal location from the service request and navigates the
 * robot to the goal location. The node uses the path planning algorithm to find
 * the shortest path to the goal location.
 *
 * Libraries
 *      Standard libraries
 *          std::string, std::vector, std::thread, std::fstream, std::cout,
 *          std::endl, std::cin, std::pow, std::sqrt, std::abs
 *      ROS libraries
 *          ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h,
 *          control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h
 *      OpenCV libraries
 *          opencv2/opencv.hpp
 * 
 * Parameters
 *      Command-line Parameters
 *          None
 *      Configuration File Parameters
 *          Key                   |     Value 
 *          --------------------- |     -------------------
 *          environmentMap        |     environmentMap.png
 *          configurationMap      |     configurationSpaceMap.png
 *          pathPlanning          |     astar
 *          socialDistance        |     true
 *          robotTopics           |     pepperTopics.dat
 *          verboseMode           |     true
 *          robotType             |     old
 *
 * Subscribed Topics and Message Types
 *      /robotLocalization/pose                 geometry_msgs/Pose2D
 *
 * Published Topics and Message Types
 *      /pepper_dcm/cmd_vel                     geometry_msgs/Twist
 *      /joint_angles                           naoqi_bridge_msgs/JointAnglesWithSpeed
 *
 * Services Invoked
 *      None
 *
 * Services Advertised and Message Types
 *      /robotNavigation/set_goal               cssr_system/setGoal
 *
 * Input Data Files
 *      pepperTopics.dat - Contains topic names for robot actuators
 *      scenarioOneEnvironmentMap.dat - Environment map data
 *      scenarioOneConfigMap.dat - Configuration space map data
 *      parameters230.dat - Locomotion parameters for old robot
 *      parameters240.dat - Locomotion parameters for new robot
 *
 * Output Data Files
 *      robotPose.dat - Updated robot position after navigation
 *      environmentMapWaypoints[Algorithm].png - Environment map with waypoints
 *      configMapWaypoints[Algorithm].png - Configuration map with waypoints
 *
 * Configuration Files
 *      robotNavigationConfiguration.ini
 *
 * Example Instantiation of the Module
 *      rosrun cssr_system robotNavigation
 *
 * Author:  Adedayo Akinade, Carnegie Mellon University Africa
 * Email:   aakinade@andrew.cmu.edu
 * Date:    January 7, 2025
 * Version: v1.0
 * 
 * Author:  Birhanu Shimelis Girma, Carnegie Mellon University Africa
 * Email:   bgirmash@andrew.cmu.edu
 * Date:    June 05, 2025
 * Version: v1.0
 */

#include "robotNavigation/robotNavigationInterface.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "robotNavigation");
    ros::NodeHandle nh;
    nodeName = ros::this_node::getName();
    if (nodeName[0] == '/') {
        nodeName = nodeName.substr(1);}
    std::string copyright_message = nodeName + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\tWebsite: www.cssr4africa.org "
                                    "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());                                                      

    ROS_INFO("%s: startup.", nodeName.c_str()); 

    ros::ServiceServer set_goal_service = nh.advertiseService("/robotNavigation/set_goal", setGoal);
    ROS_INFO("%s: Goal Server Ready to receive requests.", nodeName.c_str());

    // Read the configuration file
    int config_file_read = 0;
    topics_filename = robot_topics;
    config_file_read = readConfigurationFile(&environmentMapFile, &configurationMapFile, &pathPlanningAlgorithm, &socialDistanceMode, &robot_topics, &topics_filename, &verbose_mode, &robot_type);
    printConfiguration(environmentMapFile, configurationMapFile, pathPlanningAlgorithm, socialDistanceMode, robot_topics, topics_filename, verbose_mode, robot_type);
    
    // Check if the configuration file was read successfully
    if(config_file_read == 1){
        ROS_ERROR("Error reading the configuration file\n");
        return 0;
    }   

    /* Create a publisher object for velocity commands */
    /* ----------------------------------------------- */

    std::string wheels_topic;     // stores the wheels topic
    // Extract the topic for the wheels
    if(extractTopic("Wheels", topics_filename, &wheels_topic)){
        ROS_ERROR("Error extracting the wheels topic\n");
        return 0;
    }
    ROS_INFO("%s: Wheels topic: %s", nodeName.c_str(), wheels_topic.c_str());
    navigation_velocity_publisher = nh.advertise<geometry_msgs::Twist>(wheels_topic, 10);

    // navigation_pelvis_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/pepper_dcm/Pelvis_controller/command", 1000, true);
    navigation_pelvis_publisher = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1000, true);

    

    bool                 debug = true;
   
    FILE                 *fp_in;                    
    char                 path[MAX_FILENAME_LENGTH];
    // This code helps to get the robot_ip from ROS parameter server
    std::string robot_ip;
    bool use_ip_based_params = false;
    std::string ip_based_param_file = "parameters230.dat"; // Default
    
    if (nh.getParam("robot_ip", robot_ip)) {
        // Check if the IP matches one of our known robots
        if (robot_ip == "172.29.111.240") {
            use_ip_based_params = true;
            ip_based_param_file = "parameters240.dat";
            ROS_INFO("Detected robot with IP 172.29.111.240, will use parameters240.dat");
        } 
        else if (robot_ip == "172.29.111.230") {
            use_ip_based_params = true;
            ip_based_param_file = "parameters230.dat";
            ROS_INFO("Detected robot with IP 172.29.111.230, will use parameters230.dat");
        }
        else {
            ROS_WARN("Unknown robot IP: %s, will use configuration file setting", robot_ip.c_str());
        }
    }
    
    // Determine parameter file to use, IP-based selection takes precedence
    char locomotion_parameter_filename[MAX_FILENAME_LENGTH];
    if (use_ip_based_params) {
        strcpy(locomotion_parameter_filename, ip_based_param_file.c_str());
        ROS_INFO("Using IP-based parameter file: %s (overriding configuration)", ip_based_param_file.c_str());
    } else if (robot_type == "new") {
        strcpy(locomotion_parameter_filename, "parameters240.dat");
        ROS_INFO("%s: Using parameters for new robot (parameters240.dat) from configuration", nodeName.c_str());
    } else {
        strcpy(locomotion_parameter_filename, "parameters230.dat");
        ROS_INFO("%s: Using parameters for old robot (parameters230.dat) from configuration", nodeName.c_str());
    }
    char                 navigation_map_filename[MAX_FILENAME_LENGTH]         = "";
    char                 environment_map_filename[MAX_FILENAME_LENGTH]         = "";
    char                 navigation_pathway_filename[MAX_FILENAME_LENGTH]         = "";
    char                 path_and_input_filename[MAX_FILENAME_LENGTH]        = "";
    int                  end_of_file;
    bool                 success = true;

    double                publish_rate                = 10;   // rate at which cmd_vel commands are published

    /* Create a subscriber object for the odom topic -- */
    /* --------------------------------------------- */
    
    // if (debug) printf("Subscribing to odom\n");
    // ros::Subscriber sub = nh.subscribe("/naoqi_driver/odom", 1, &odomMessageReceived);
    // Check if /robotLocalization/pose topic is available
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    bool pose_topic_found = false;
    for (const auto& topic : master_topics) {
        if (topic.name == "/robotLocalization/pose") {
            pose_topic_found = true;
            break;
        }
    }

    ros::Subscriber sub;  // Keep this declaration
    if (pose_topic_found) {
        ROS_INFO("%s: Subscribing to /robotLocalization/pose", nodeName.c_str());
        sub = nh.subscribe("/robotLocalization/pose", 1, &poseMessageReceived);  // ← REMOVE "ros::Subscriber"
    } else {
        ROS_INFO("%s: Subscribing to /naoqi_driver/odom", nodeName.c_str());
        // sub = nh.subscribe("/naoqi_driver/odom", 1, &odomMessageReceived);  // ← Add actual odom subscription
    }
    
    // Verify subscription worked
    if (sub) {
        ROS_INFO("%s: Successfully subscribed to pose topic", nodeName.c_str());
    } else {
        ROS_ERROR("%s: Failed to subscribe to pose topic", nodeName.c_str());
    }
    // ros::Subscriber sub = nh.subscribe("/robotLocalization/pose", 1, &poseMessageReceived);


    // /* construct the full path and filename */
    // /* ------------------------------------ */
    
    packagedir = ros::package::getPath(ROS_PACKAGE_NAME);

    /* get the dimensions of the environment map in centimeters */
    /* -------------------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, environmentMapFile.c_str());

    mapImage = imread(path_and_input_filename, IMREAD_GRAYSCALE);


    /* get the dimensions of the navigation map in centimeters */
    /* -------------------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, configurationMapFile.c_str());

    configurationSpaceImage = imread(path_and_input_filename, IMREAD_GRAYSCALE);

    x_map_size = configurationSpaceImage.cols;
    y_map_size = configurationSpaceImage.rows;

    // Get image dimensions (room dimensions based on image size)
    image_width = configurationSpaceImage.cols;
    image_height = configurationSpaceImage.rows;
    // printf("Image width: %d, Image height: %d\n", image_width, image_height);

    room_width = (double) image_width / 100;  // Width of the room in meters
    room_height = (double) image_height / 100;  // Height of the room in meters
    // printf("Room width: %.2f, Room height: %.2f\n", room_width, room_height);

    /* get the locomotion parameter data */
    /* --------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, locomotion_parameter_filename);
    
    readLocomotionParameterData(path_and_input_filename, &locomotionParameterData);

    /* convert map image to a graph  */
    /* ----------------------------- */

    buildGraphFromMap(configurationSpaceImage, graph, pathPlanningAlgorithm);

    if(locomotionParameterData.robot_available){
        moveRobotActuatorsToDefault();
    }

    while(ros::ok()){       
        ROS_INFO_THROTTLE(10, "%s: running.", nodeName.c_str());
        ros::spinOnce(); 

    }

    return 0;
}
