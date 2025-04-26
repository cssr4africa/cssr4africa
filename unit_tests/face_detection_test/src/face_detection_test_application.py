#!/usr/bin/env python3

"""
face_detection_test_application.py Application code to run the Face and Mutual Gaze Detection and Localization Unit test.

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
face_detection_test_application.py  Application code to run the Face and Mutual Gaze Detection and Localization unit test.

This face_detection_test is a unit test application code to test the face and mutual gaze detection algorithm.
This code contains the main function that initializes the correct configuration parameters and tests face 
detection algorthim. It has also utility functions to save video and images with the bounding boxes and gaze 
detection. The face detection algorithm can be either MediaPipe Face Detection or SixDrepNet that can be
configured from the configuration file. 

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
    - face_detection_test_msg_file.msg (msg_file)

Parameters
    Launch File Parameters:
        roslaunch unit_tests face_detection_test_launch_robot.launch camera:=realsense
            camera: Camera type or video file (realsense or pepper or video)
            bag_file: ROS bag file for testing (singleFace, multipleFaces, faceTracking, mutualGaze, occlusion, lighting)
            pepper_robot_ip: Pepper robot IP address (e.g 172.29.111.230 or 172.29.111.240)
            network_interface: Network interface for Pepper robot connection

        roslaunch unit_tests face_detection_test_launch_test_harness.launch

    Configuration File Parameters
        Key                             Value
        algorithm                       sixdrep
        useCompressed                   false
        saveVideo                       false
        saveImage                       false
        videoDuration                   10
        imageInterval                   5
        recordingDelay                  5
        max_frames_buffer               300
        verboseMode                     false

Subscribed Topics
    Topic Name                                  Message Type
    /camera/color/image_raw                     sensor_msgs/Image              
    /camera/aligned_depth_to_color/image_raw    sensor_msgs/Image
    /naoqi_driver/camera/front/image_raw        sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw        sensor_msgs/Image
    /faceDetection/data                         face_detection/face_detection_msg_file.msg

Published Topics
    None

Input Data Files
    - pepperTopics.dat: Data file for Pepper robot camera topics
    - face_detection_test_input_realsense_single_face.bag
    - face_detection_test_input_realsense_multiple_faces.bag
    - face_detection_test_input_realsense_mutual_gaze.bag
    - face_detection_test_input_realsense_lighting_1.bag
    - face_detection_test_input_realsense_lighting_2.bag

Output Data Files
    - face_detection_test_rgb_video_{start_time}.mp4
    - face_detection_test_depth_video_{start_time}.mp4
    - face_detection_test_rgb_image_{start_time}.png
    - face_detection_test_depth_image_{start_time}.png

Configuration File
    face_detection_test_configuration.json

Example of instantiation of the module
    roslaunch unit_tests face_detection_test_launch_robot.launch camera:=video bag_file:=single_face
    
    (In a new terminal)
    roslaunch unit_tests face_detection_test_launch_test_harness.launch

Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: March 21, 2025
Version: v1.0
"""

import rospy
from face_detection_test_implementation import FaceDetectionTest

def main():
    # Define the node name and software version
    node_name = "faceDetectionTest"
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
    rospy.init_node(node_name)

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")

    # Create the test instance and run tests
    FaceDetectionTest()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == '__main__':
    main()
