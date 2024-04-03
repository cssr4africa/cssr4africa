/* sensorTestApplication.cpp
*
* <detailed functional description>
* The component test the functionality of the sensor of the robot using the ROS interface.
* The test is performed by subscribing to the sensor topics and checking if the robot sends
* the expected data. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously.
...

* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::fabs, std::time_t, std::tm, std::localtime, std::strftime
* ROS libraries
- ros/ros.h, ros/package.h, image_transport/image_transport.h, sensor_msgs/CameraInfo.h, sensor_msgs/Range.h, sensor_msgs/JointState.h, sensor_msgs/LaserScan.h, cv_bridge/cv_bridge.h, opencv2/highgui/highgui.hpp

...
* Parameters
*
* Command-line Parameters
...
* Configuration File Parameters

* Key | Value 
* --- | ---
* platform        | robot
* simulatorTopics | simulatorTopics.dat
* robotTopics     | pepperTopics.dat
* mode            | sequential

* Key | Value
* --- | ---
* BackSonar     |   true
* FrontSonar    |   true    
* BottomCamera  |   true
* FrontCamera   |   true
* DepthCamera   |   true
* StereoCamera  |   true
* LaserSensor   |   true
* Microphone    |   true
* JointState    |   true
* Odometry      |   true
* IMU           |   true
* Speech        |   true
...
* Subscribed Topics and Message Types
*
* /naoqi_driver/sonar/back                      sensor_msgs/Range                 
* /naoqi_driver/sonar/front                     sensor_msgs/Range
* /naoqi_driver/camera/front/image_raw          sensor_msgs/Image
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
...
* Published Topics and Message Types
* /naoqi_driver/speech                          std_msgs/String
*
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* sensorTestOutput.dat, 
* frontCameraOutput.mp4,
* bottomCameraOutput.mp4,
* depthCameraOutput.mp4,
* stereoCameraOutput.mp4,
* microphoneOutput.wav
...
* Configuration Files
*
* sensorTestConfiguration.ini
* actuatorTestInput.ini
...
* Example Instantiation of the Module
*
* rosrun pepper_interface_tests sensorTest
...
*
* Author: Yohannes Tadesse Haile, Carnegie Mellon University Africa
* Email: yohanneh@andrew.cmu.edu
* Date: March 19, 2024
* Version: v1.0
*
*/

# include "pepper_interface_tests/sensorTest.h"

/* Main function */
int main(int argc, char **argv){
    // Initialize the ROS node
    ros::init(argc, argv, "sensorTest");
    ros::NodeHandle nh;

    std::vector<std::string> testNames = extractTests("sensor");
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
    return 0;
}