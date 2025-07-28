#!/usr/bin/env python3

"""
person_detection_test_application.py Application code to run the person detection and localization unit test.

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
person_detection_test_application.py  Application code to run person detection and localization unit test.

This person_detection_test is a unit test application code to test the person detection and localization algorithm.
This code contains the main function that initializes the correct configuration parameters and tests whether person is 
detected and localized. It has also utility functions to save video and images with the bounding boxes. 

Libraries
    - rospkg
    - rospy
    - os
    - json
    - numpy
    - cv2
    - time
    - threading
    - colorsys
    - sensor_msgs.msg (Image)
    - cv_bridge (CvBridge)
    - message_filters (ApproximateTimeSynchronizer, Subscriber)
    - unit_tests.msg (person_detection_test_msg_file)

Parameters
    Launch File Parameters:
        roslaunch unit_tests person_detection_test_launch_robot.launch camera:=realsense
            camera: Camera type or video file (realsense or pepper or video)
            bag_file: ROS bag file for testing 
            pepper_robot_ip: Pepper robot IP address (e.g 172.29.111.230 or 172.29.111.240)
            network_interperson: Network interperson for Pepper robot connection

        roslaunch unit_tests person_detection_test_launch_test_harness.launch

Configuration File Parameters
    Key                                                     Value
    saveVideo                                               false
    saveImage                                               false
    videoDuration                                           10
    imageInterval                                           5
    recordingDelay                                          5
    maxFramesBuffer                                         300
    verboseMode                                             false

Subscribed Topics
    Topic Name                                              Message Type
    /camera/color/image_raw                                 sensor_msgs/Image             
    /camera/aligned_depth_to_color/image_raw                sensor_msgs/Image
    /naoqi_driver/camera/front/image_raw                    sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw                    sensor_msgs/Image
    /personDetection/data                                   person_detection/person_detection_test_msg_file.msg

Published Topics
    None

Input Data Files
    - pepperTopics.dat: Data file for Pepper robot camera topics
    - person_detection_test_input_realsense_single_person.bag
    - person_detection_test_input_realsense_multiple_people.bag
    - person_detection_test_input_realsense_lighting_1.bag
    - person_detection_test_input_realsense_lighting_2.bag

Output Data Files
    - person_detection_test_rgb_video_{start_time}.mp4
    - person_detection_test_depth_video_{start_time}.mp4
    - person_detection_test_rgb_image_{start_time}.png
    - person_detection_test_depth_image_{start_time}.png

Configuration File
    person_detection_test_configuration.json

Example of instantiation of the module
    roslaunch unit_tests person_detection_test_launch_robot.launch camera:=video bag_file:=single_person
    
    (In a new terminal)
    roslaunch unit_tests person_detection_test_launch_testHarness.launch

Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: March 21, 2025
Version: v1.0
"""

import rospy
from person_detection_test_implementation import PersonDetectionTest

def main():
    # Define the node name and software version
    node_name = "personDetectionTest"
    software_version = "v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
        "\t\t\t    Website: www.cssr4africa.org\n"
        "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
    )

    # Initialize the ROS node
    rospy.init_node(node_name, anonymous=True)

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")

    # Create the test instance and run tests
    PersonDetectionTest()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == '__main__':
    main()