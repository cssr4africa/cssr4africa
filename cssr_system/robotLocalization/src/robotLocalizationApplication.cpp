/* robotLocalizationApplication.cpp    Program initialization and main function execution
*
* Author:  Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email:   ioj@andrew.cmu.edu
* Date:    June 25, 2025
* Version: v1.0
*
* Copyright (C) 2025 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.

* This node is responsible for determining the robot's absolute position and orientation in the environment using
* visual landmark detection and sensor fusion. The node combines ArUco marker detection from RGB and depth cameras with
* odometry data to provide accurate 6-DOF pose estimation. The system uses triangulation (RGB-only) or trilateration
* (with depth) algorithms to compute absolute poses from detected landmarks, then maintains relative positioning
* through odometry integration. The node supports both periodic automatic pose correction and on-demand pose reset
* services for robust localization in dynamic environments.
*
* Libraries
*     Standard libraries
*        std::string, std::vector, std::map, std::fstream, std::algorithm, std::numeric, std::sqrt, std::abs,
*        std::atan2, std::cos, std::sin
*     ROS libraries
*        ros/ros.h, nav_msgs/Odometry.h, sensor_msgs/Imu.h, sensor_msgs/Image.h, sensor_msgs/JointState.h,
*        sensor_msgs/CameraInfo.h, geometry_msgs/Pose2D.h, geometry_msgs/TransformStamped.h
*     ROS TF libraries
*        tf2/LinearMath/Quaternion.h, tf2_geometry_msgs/tf2_geometry_msgs.h, tf2_ros/transform_listener.h,
*        tf2_ros/buffer.h
*     OpenCV libraries
*        opencv2/opencv.hpp, opencv2/aruco.hpp, cv_bridge/cv_bridge.h
*     Image transport
*        image_transport/image_transport.h
*     Utility libraries
*        angles/angles.h, yaml-cpp/yaml.h, json/json.h
* 
* Parameters
*     Command-line Parameters
*        None
*     Configuration File Parameters
*        Key                  |     Value 
*        -------------------- |     -------------------
*        verboseMode          |     false
*        camera               |     FrontCamera
*        depthCamera          |     DepthRealSense
*        useDepth             |     false
*        resetInterval        |     30.0
*        absolutePoseTimeout  |     300.0
*        cameraInfoTimeout    |     10.0
*        useHeadYaw           |     false
*        headYawJointName     |     HeadYaw
*        mapFrame             |     map
*        odomFrame            |     odom
*        landmarkFile         |     /robotLocalization/data/arucoLandmarks.json
*        topicsFile           |     /robotLocalization/data/pepperTopics.dat
*        cameraInfoFile       |     /robotLocalization/data/cameraInfo.yaml
*
* Subscribed Topics and Message Types
*     /naoqi_driver/camera/front/image_raw         sensor_msgs/Image
*     /naoqi_driver/camera/stereo/image_raw        sensor_msgs/Image
*     /camera/color/image_raw                      sensor_msgs/Image
*     /camera/aligned_depth_to_color/image_raw     sensor_msgs/Image
*     /camera/color/camera_info                    sensor_msgs/CameraInfo
*     /naoqi_driver/odom                           nav_msgs/Odometry
*     /naoqi_driver/imu/base                       sensor_msgs/Imu
*     /joint_states                                sensor_msgs/JointState
*
* Published Topics and Message Types
*     /robotLocalization/pose                      geometry_msgs/Pose2D
*     /robotLocalization/marker_image              sensor_msgs/Image
*
* Services Invoked
*     None
*
* Services Advertised and Message Types
*     /robotLocalization/reset_pose                cssr_system/ResetPose
*     /robotLocalization/set_pose                  cssr_system/SetPose
*
* Input Data Files
*     pepperTopics.dat        Contains topic names for robot sensors and actuators
*     arucoLandmarks.json     3D coordinates of ArUco markers in the environment
*     cameraInfo.json         Camera intrinsic parameters for fallback (fx, fy, cx, cy)
*
* Output Data Files
*     None (publishes pose data via ROS topics)
*
* Configuration Files
*     robotLocalizationConfiguration.json - Main configuration parameters file
*
* Example Instantiation of the Module
*     roslaunch cssr_system robotLocalizationLaunchRobot.launch
*
* Key Algorithms
*     Triangulation Algorithm (RGB-only mode):
*        - Detects ArUco markers in camera image
*        - Computes viewing angles between marker pairs using camera intrinsics
*        - Uses geometric triangulation with circle-circle intersection to determine robot position
*        - Employs multi-solution scoring system based on geometric constraints
*
*     Trilateration Algorithm (Depth mode):
*        - Combines ArUco detection with depth measurements
*        - Solves system of distance equations to determine robot position
*        - Uses least-squares approach for overdetermined systems
*
* Pose Fusion:
*     - Maintains baseline pose from absolute measurements
*     - Integrates odometry for continuous pose updates
*     - Applies periodic corrections to prevent drift accumulation
*
* Marker Detection Pipeline:
*     - Sorts detected markers by position or ID for consistent processing
*     - Validates marker configurations for geometric feasibility
*     - Rejects degenerate cases (collinear markers, extreme angles)
*     - Publishes annotated images showing detected markers
*
* Author:   Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email:    ioj@andrew.cmu.edu
* Date:     June 25, 2025
* Version:  v1.0
*
*/

#include "robotLocalization/robotLocalizationInterface.h"

// Constructor
RobotLocalizationNode::RobotLocalizationNode() : nh_("~"), it_(nh_), tf_buffer_(), tf_listener_(tf_buffer_) {
    
    // Dynamically resolve config file path in the package
    std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string config_path = package_path + "/robotLocalization/config/robotLocalizationConfiguration.json";

    // Load and parse the JSON configuration file
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        ROS_ERROR("Could not open JSON config file at: %s", config_path.c_str());
        ros::shutdown();
        return;
    }

    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(config_file, config)) {
        ROS_ERROR("Failed to parse JSON config file");
        ros::shutdown();
        return;
    }

    // Load configuration parameters
    verbose_ = config.get("verboseMode", false).asBool();
    camera_ = config.get("camera", "FrontCamera").asString();
    depth_camera_ = config.get("depthCamera", "DepthRealSense").asString();
    use_depth_ = config.get("useDepth", false).asBool();
    reset_interval_ = config.get("resetInterval", 30.0).asDouble();
    absolute_pose_timeout_ = config.get("absolutePoseTimeout", 300.0).asDouble();
    camera_info_timeout_ = config.get("cameraInfoTimeout", 10.0).asDouble();
    use_head_yaw_ = config.get("useHeadYaw", true).asBool();
    head_yaw_joint_name_ = config.get("headYawJointName", "HeadYaw").asString();
    map_frame_ = config.get("mapFrame", "map").asString();
    odom_frame_ = config.get("odomFrame", "odom").asString();
    landmark_file_ = package_path + config.get("landmarkFile", "").asString();
    topics_file_ = package_path + config.get("topicsFile", "").asString();
    camera_info_file_ = package_path + config.get("cameraInfoFile", "").asString();

    // Load topics and landmarks
    loadTopicNames();
    loadLandmarks();

    // Subscribers
   odom_sub_ = nh_.subscribe(topic_map_["Odometry"], 10, &RobotLocalizationNode::odomCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_["Odometry"].c_str());

   imu_sub_ = nh_.subscribe(topic_map_["IMU"], 10, &RobotLocalizationNode::imuCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_["IMU"].c_str());

   image_sub_ = it_.subscribe(topic_map_[camera_], 10, &RobotLocalizationNode::imageCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_[camera_].c_str());

   depth_sub_ = it_.subscribe(topic_map_[depth_camera_], 10, &RobotLocalizationNode::depthCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_[depth_camera_].c_str());

   joint_sub_ = nh_.subscribe(topic_map_["HeadYaw"], 10, &RobotLocalizationNode::jointCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_["HeadYaw"].c_str());

   camera_info_sub_ = nh_.subscribe(topic_map_["CameraInfo"], 1, &RobotLocalizationNode::cameraInfoCallback, this);
   ROS_INFO("robotLocalization: subscribed to %s", topic_map_["CameraInfo"].c_str());

    // Publishers
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
    image_pub_ = it_.advertise("/robotLocalization/marker_image", 10);

    // Service
    reset_srv_ = nh_.advertiseService("/robotLocalization/reset_pose", &RobotLocalizationNode::resetPoseCallback, this);
    setpose_srv_ = nh_.advertiseService("/robotLocalization/set_pose", &RobotLocalizationNode::setPoseCallback, this);

    initializePoseAdjustments();

    // Initialize pose
    current_pose_.x = 0.0;
    current_pose_.y = 0.0;
    current_pose_.theta = 0.0;
    baseline_pose_ = current_pose_;
    last_odom_pose_ = current_pose_;
    last_absolute_pose_time_ = ros::Time(0);

    // Initialize other variables
    camera_info_received_ = false;
    first_odom_received_ = false;
    head_yaw_ = 0.0;
    camera_height_ = 1.225;
    fx_ = 0.0; fy_ = 0.0; cx_ = 0.0; cy_ = 0.0;
    initial_robot_x = 0.0; initial_robot_y = 0.0; initial_robot_theta = 0.0;
    adjustment_x_ = 0.0; adjustment_y_ = 0.0; adjustment_theta_ = 0.0;
    odom_x_ = 0.0; odom_y_ = 0.0; odom_theta_ = 0.0;

    // Timer for periodic reset
    reset_timer_ = nh_.createTimer(ros::Duration(reset_interval_), &RobotLocalizationNode::resetTimerCallback, this);

    // Timer for camera info timeout
    camera_info_timer_ = nh_.createTimer(ros::Duration(camera_info_timeout_), &RobotLocalizationNode::cameraInfoTimeoutCallback, this, true);

    // Heartbeat timer (every 10 seconds)
    heartbeat_timer_ = nh_.createTimer(ros::Duration(10.0), 
      [this](const ros::TimerEvent&) {
         ROS_INFO("robotLocalization: running.");
      });

    ROS_INFO("robotLocalization node initialized");
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "robotLocalization");

    // Get node name for logging
    std::string nodeName = ros::this_node::getName();
    if (nodeName[0] == '/') {
        nodeName = nodeName.substr(1);
    }
    
    // Copyright message on startup
    std::string copyrightMessage = nodeName + ": " + std::string(SOFTWARE_VERSION) + 
                                    "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
                                    "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme. "
                                    "\n\t\t\t\tWebsite: www.cssr4africa.org "
                                    "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";
    
    ROS_INFO("%s", copyrightMessage.c_str());
    ROS_INFO("%s: start-up.", nodeName.c_str());
    
    // Create the robot localization node
    RobotLocalizationNode node;

    // Spin to process callbacks
    ros::spin();

    
    // Clean up OpenCV windows if created
    cv::destroyAllWindows();
    
    return 0;
}