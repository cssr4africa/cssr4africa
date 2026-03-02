#include "robotLocalization/robotLocalizationInterface.h"
#include <fstream>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>

// Implementation of RobotLocalization methods
RobotLocalization::RobotLocalization() {
    loadConfiguration();
    relative_pose.x = 4.4;
    relative_pose.y = 7.8;
    relative_pose.theta = 270.0;

    process_image = false;
    first_odom_received_ = false;
    adjustment_x_ = adjustment_y_ = adjustment_theta_ = 0.0;
    odom_x_ = odom_y_ = odom_theta_ = 0.0;
    initROS();
    initializePoseAdjustments();
}

void RobotLocalization::loadConfiguration() {
    // Load configuration file
    ros::NodeHandle private_nh("~");
    std::string config_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/config/robotLocalizationConfiguration.ini";
    std::ifstream infile(config_file);
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open configuration file: %s", config_file.c_str());
        ros::shutdown();
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        config_[key] = value;
    }
    infile.close();

    // Set configuration variables
    platform_ = config_["platform"];
    camera_ = config_["camera"];
    reset_interval_ = std::stoi(config_["resetInterval"]);
    robot_topics_file_ = config_["robotTopics"];
    simulator_topics_file_ = config_["simulatorTopics"];
    verbose_mode = (config_["verboseMode"] == "true");

    // Load topic names
    loadTopicNames();
}

void RobotLocalization::loadTopicNames() {
    std::string topics_file;
    if (platform_ == "robot")
        topics_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/data/" + robot_topics_file_;
    else
        topics_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/data/" + simulator_topics_file_;

    std::ifstream infile(topics_file.c_str());
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open topics file: %s", topics_file.c_str());
        ros::shutdown();
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        topics_[key] = value;
    }
    infile.close();
}

void RobotLocalization::initROS() {
    odom_sub = nh.subscribe(topics_["Odometry"], 10, &RobotLocalization::odomCallback, this);
    imu_sub = nh.subscribe(topics_["IMU"], 10, &RobotLocalization::imuCallback, this);
    joint_state_sub = nh.subscribe(topics_["HeadYaw"], 10, &RobotLocalization::jointStateCallback, this);

    image_transport::ImageTransport it(nh);
    camera_sub = it.subscribe(topics_[camera_], 1, &RobotLocalization::imageCallback, this);

    pose_pub_ = nh.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
    reset_service = nh.advertiseService("/robotLocalization/set_pose", &RobotLocalization::setPoseService, this);
}

void RobotLocalization::initializePoseAdjustments() {
    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = angles::normalize_angle(initial_robot_theta - odom_theta_);
}

void RobotLocalization::setInitialValues(double x, double y, double theta) {
    // ROS_INFO("Setting initial robot pose to (x: %.2f, y: %.2f, theta: %.2f degrees)", x, y, theta);
    
    initial_robot_x = x;
    initial_robot_y = y;
    // Convert theta from degrees to radians, just like in setPoseService
    initial_robot_theta = angles::from_degrees(theta);
    
    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // Ensure odometry data is up-to-date
    ros::spinOnce();
    
    // Calculate adjustments between initial pose and current odometry
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;
    
    // ROS_INFO("Initial pose set successfully - Adjustments calculated (x_adj: %.2f, y_adj: %.2f, theta_adj: %.2f rad)", adjustment_x_, adjustment_y_, adjustment_theta_);
}

void RobotLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_theta_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    first_odom_received_ = true;

    double x = odom_x_ + adjustment_x_ - initial_robot_x;
    double y = odom_y_ + adjustment_y_ - initial_robot_y;

    double current_x = x * cos(adjustment_theta_) - y * sin(adjustment_theta_);
    double current_y = x * sin(adjustment_theta_) + y * cos(adjustment_theta_);

    current_x += initial_robot_x;
    current_y += initial_robot_y;

    double current_theta = odom_theta_ + adjustment_theta_;

    relative_pose.x = current_x;
    relative_pose.y = current_y;
    relative_pose.theta = current_theta;

    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = relative_pose.x;
    pose_msg.y = relative_pose.y;
    pose_msg.theta = angles::to_degrees(relative_pose.theta);

    pose_pub_.publish(pose_msg);

    if (verbose_mode) {
        ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f degrees", current_x, current_y, angles::to_degrees(current_theta));
    }
}

void RobotLocalization::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Process IMU data if needed
}

void RobotLocalization::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "HeadYaw") {
            head_yaw_angle_ = msg->position[i];
            break;
        }
    }
}

void RobotLocalization::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr_->image;

        if (process_image) {
            recognizeLandmarks();
            process_image = false;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool RobotLocalization::setPoseService(cssr_system::setPose::Request& req, cssr_system::setPose::Response& res) {
    initial_robot_x = req.x;
    initial_robot_y = req.y;
    initial_robot_theta = angles::from_degrees(req.theta);

    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;
    ROS_INFO("Pose reset to (%5.3f, %5.3f, %5.3f degrees)", initial_robot_x, initial_robot_y, angles::to_degrees(initial_robot_theta));
    ROS_INFO("Adjustments: (%5.3f, %5.3f, %5.3f degrees)", adjustment_x_, adjustment_y_, angles::to_degrees(adjustment_theta_));
    res.success = true;
    return true;
}

void RobotLocalization::recognizeLandmarks() {
    relative_pose.x = 0.0;
    relative_pose.y = 0.0;
    relative_pose.theta = 0.0;
}

std::string RobotLocalization::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}
