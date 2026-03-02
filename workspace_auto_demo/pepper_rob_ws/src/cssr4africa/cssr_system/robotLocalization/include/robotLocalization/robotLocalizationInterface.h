#ifndef ROBOT_LOCALIZATION_INTERFACE_H
#define ROBOT_LOCALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include "cssr_system/setPose.h"

class RobotLocalization {
public:
    RobotLocalization();
    void setInitialValues(double x, double y, double theta);

private:
    void loadConfiguration();
    void loadTopicNames();
    void initROS();
    void initializePoseAdjustments();
    void performAbsolutePoseEstimation();
    void recognizeLandmarks();
    void publishPose();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool setPoseService(cssr_system::setPose::Request& req, cssr_system::setPose::Response& res);

    std::string trim(const std::string& str);

    // ROS Components
    ros::NodeHandle nh;
    ros::Subscriber odom_sub, imu_sub, joint_state_sub;
    image_transport::Subscriber camera_sub;
    ros::Publisher pose_pub_;
    ros::ServiceServer reset_service;

    // Member Variables
    geometry_msgs::Pose2D relative_pose, last_reset_pose;
    nav_msgs::Odometry previous_odom_;

    double head_yaw_angle_;
    bool process_image, first_odom_received_;

    std::map<std::string, std::string> config_;
    std::map<std::string, std::string> topics_;
    std::string platform_, camera_, robot_topics_file_, simulator_topics_file_;
    int reset_interval_;
    bool verbose_mode;

    cv::Mat current_image_;
    cv_bridge::CvImagePtr cv_ptr_;

    double initial_robot_x, initial_robot_y, initial_robot_theta;
    double adjustment_x_, adjustment_y_, adjustment_theta_;
    double odom_x_, odom_y_, odom_theta_;
};

#endif
