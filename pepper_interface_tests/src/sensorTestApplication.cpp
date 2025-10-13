/* sensorTestApplication.cpp Application to test the sensors of the Pepper robot using ROS interface.   
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
* The component test the functionality of the sensor of the robot using the ROS interface.
* The test is performed by subscribing to the sensor topics and checking if the robot sends
* the expected data. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously.

* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::fabs, std::time_t, std::tm, std::localtime, std::strftime
* ROS libraries
- ros/ros.h, ros/package.h, image_transport/image_transport.h, sensor_msgs/CameraInfo.h, sensor_msgs/Range.h, sensor_msgs/JointState.h, sensor_msgs/LaserScan.h, cv_bridge/cv_bridge.h, opencv2/highgui/highgui.hpp

* Parameters
*
* Command-line Parameters

* Configuration File Parameters

* Key | Value 
* --- | ---
* platform        | robot
* simulatorTopics | simulatorTopics.dat
* robotTopics     | pepperTopics.dat
* mode            | sequential

* Key | Value
* --- | ---
* BackSonar             |   true
* FrontSonar            |   true    
* BottomCamera          |   true
* FrontCamera           |   true
* realsenseRGBDCamera   |   true
* realsenseDepthCamera  |   true
* DepthCamera           |   true
* StereoCamera          |   true
* LaserSensor           |   true
* Microphone            |   true
* JointState            |   true
* Odometry              |   true
* IMU                   |   true
* Speech                |   true

* Subscribed Topics and Message Types
*
* /naoqi_driver/sonar/back                      sensor_msgs/Range                 
* /naoqi_driver/sonar/front                     sensor_msgs/Range
* /naoqi_driver/camera/front/image_raw          sensor_msgs/Image
* /camera/color/image_raw                       sensor_msgs/Image
* /camera/aligned_depth_to_color/image_raw      sensor_msgs/Image
* /naoqi_driver/camera/bottom/image_raw         sensor_msgs/Image
* /naoqi_driver/camera/depth/image_raw          sensor_msgs/Image
* /naoqi_driver/laser                           sensor_msgs/LaserScan
* /naoqi_driver/audio                           naoqi_driver/AudioCustomMsg
* /naoqi_driver/joint_states                    sensor_msgs/JointState
* /naoqi_driver/odom                            nav_msgs/Odometry
* /naoqi_driver/imu                             sensor_msgs/Imu

* /pepper/sonar_back                            sensor_msgs/Range
* /pepper/sonar_front                           sensor_msgs/Range
* /pepper/camera/front/image_raw                sensor_msgs/Image
* /pepper/camera/bottom/image_raw               sensor_msgs/Image
* /pepper/camera/depth/image_raw                sensor_msgs/Image
* /pepper/laser_2                               sensor_msgs/LaserScan
* /joint_states                                 sensor_msgs/JointState
* /pepper/odom                                  nav_msgs/Odometry

* Published Topics and Message Types
* /naoqi_driver/speech                          std_msgs/String

* Services Invoked
*
* None

* Services Advertised and Request Message
* 
* None

* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
* sensorTestInput.dat

* Output Data Files
*
* sensorTestOutput.dat, 
* frontCameraOutput.mp4,
* realsenseRGBOutput.mp4,
* realsenseDepthOutput.mp4,
* bottomCameraOutput.mp4,
* depthCameraOutput.mp4,
* stereoCameraOutput.mp4,
* microphoneOutput.wav

* Configuration Files
*
* sensorTestConfiguration.ini

* Example Instantiation of the Module
*
* rosrun pepper_interface_tests sensorTest

*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa, Carnegie Mellon University Africa
* Email: yohanneh@andrew.cmu.edu
* Date: October 13, 2025
* Version: v1.1
*
*/

# include "pepper_interface_tests/sensorTestInterface.h"
#include <X11/Xlib.h>

/* Main function */
int main(int argc, char **argv){

    XInitThreads(); // To avoid potential X11 multithreading issues with OpenCV

    // Initialize the ROS node
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

   // Start an async spinner so timers/callbacks keep firing even if tests block
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Get the name of the node (without leading '/')
    const std::string node_name = cleanNodeName(ros::this_node::getName());

    // Heartbeat every 10 seconds
    ros::Timer heartbeat = nh.createTimer(ros::Duration(10.0), heartbeatCb);

    std::string software_version = "v1.1";

    std::string copyright_message =
        " " + node_name + ": " + software_version +
        "\n\t\t\t\tThis project is funded by the African Engineering and Technology Network (Afretec)"
        "\n\t\t\t\tInclusive Digital Transformation Research Grant Programme."
        "\n\t\t\t\tWebsite: www.cssr4africa.org"
        "\n\t\t\t\tThis program comes with ABSOLUTELY NO WARRANTY.";

    ROS_INFO("%s", copyright_message.c_str());

    ROS_INFO(" %s: startup.", node_name.c_str());                                                    // Print the copyright message

    ros::NodeHandle pnh("~");
    std::string camera = pnh.param<std::string>("/camera", "both");
    std::cout<<"Camera parameter: "<<camera<<std::endl;

    std::vector<std::string> testNames = extractTests(camera);

    std::string mode = extractMode();

    std::string path = getOutputFilePath();

    std::ofstream out_of;
    initializeOutputFile(out_of, path);
    
    if (mode == "parallel") {
        executeTestsInParallel(testNames, nh);
    } else if (mode == "sequential") {
        executeTestsSequentially(testNames, nh);
    } else {
        std::cerr << "Invalid mode. Please check the mode in the configuration file.\n";
        promptAndExit(1);
    }

    finalizeOutputFile(out_of, path);
    ros::Duration(0.1).sleep();

    return 0;
}