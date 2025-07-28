#!/usr/bin/env python3

"""
face_detection_application.py Application code to run the Face and Mutual Gaze Detection and Localization ROS node.

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
face_detection_application.py   Application code to run the face and mutual gaze detection algorithm.

The face detection is implemented using the ROS image topic that could be configured to be the intel realsense camera or pepper robot
camera. It uses OpenCV to visualize the detected faces and gaze direction. The gaze direction is calculated using face
mesh landmarks which uses Google's MediaPipe library. The media pipe utilizes CPU for face detection and gaze direction.
The SixDrepNet uses YOLOONNX for face detection and SixDrepNet for gaze direction. The SixDrepNet utilizes GPU for faster
inference and better performance. This code contains the main function that initializes the face detection node and 
starts the face detection algorithm. The face detection algorithm can be either MediaPipe Face Detection or SixDrepNet 
that can be configured from the configuration file. It is also responsible for detecting the head pose estimation of the 
detected face. It subscribes to the intel realsense camera or pepper robot camera topics for the RGB and depth images.
It publishes three one topic: /faceDetection/data that contains the face label ID, the centroid of the face, 
mutual gaze direction. 

Libraries
    - cv2
    - mediapipe
    - numpy
    - rospy
    - rospkg
    - os
    - onnxruntime
    - multiprocessing
    - json
    - random
    - math (cos, sin, pi)
    - sensor_msgs.msg (Image, CompressedImage)
    - cv_bridge (CvBridge, CvBridgeError)
    - message_filters (ApproximateTimeSynchronizer, Subscriber)
    - geometry_msgs.msg (Point)
    - typing (Tuple, List)
    - cssr_system.msg (face_detection_msg_file)
    - face_detection_tracking (Sort, CentroidTracker)
    
Parameters
    Launch File Parameters:
        roslaunch cssr_system face_detection_robot.launch camera:=realsense
            camera: Camera type or video file (realsense or pepper or video)
            pepper_robot_ip: Pepper robot IP address (e.g 172.29.111.230 or 172.29.111.240)
            network_interface: Network interface for Pepper robot connection

Configuration File Parameters
    Key                                                     Value
    algorithm                                               sixdrep
    useCompressed                                           true
    mpFacedetConfidence                                     0.5
    mpHeadposeAngle                                         5
    centroidMaxDistance                                     15
    centroidMaxDisappeared                                  100
    sixdrepnet_confidence                                   0.65
    sixdrepnetHeadposeAngle                                 10
    sortMaxDisappeared                                      30
    sortMinHits                                             20
    sortIouThreshold                                        0.3
    verboseMode                                             true

Subscribed Topics
    Topic Name                                              Message Type
    /camera/color/image_raw                                 sensor_msgs/Image
    /camera/color/image_raw/compressed                      sensor_msgs/CompressedImage                 
    /camera/aligned_depth_to_color/image_raw                sensor_msgs/Image
    /camera/aligned_depth_to_color/image_raw/compressed     sensor_msgs/CompressedImage
    /naoqi_driver/camera/front/image_raw                    sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw                    sensor_msgs/Image

Published Topics
    Topic Name                                              Message Type
    /faceDetection/data                                     face_detection/face_detection_msg_file.msg

Input Data Files
    - pepperTopics.dat: Data file for Pepper robot camera topics

Model Files
    - face_detection_YOLO.onnx: YOLOONNX model for face detection
    - face_detection_sixdrepnet360.onnx: SixDrepNet model for gaze direction

Output Data Files
    None

Configuration File
    face_detection_configuration.json

Example of instantiation of the module
    roslauch cssr_system face_detection_robot.launch camera:=realsense

    # Activate the python environment
    source ~/workspace/pepper_rob_ws/cssr4africa_face_person_detection_env/bin/activate

    (In a new terminal)
    rosrun cssr_system face_detection_application.py

Author: Yohannes Tadesse Haile, Carnegie Mellon University Africa
Email: yohanneh@andrew.cmu.edu
Date: April 18, 2025
Version: v1.0
"""

import rospy
from face_detection_implementation import MediaPipe, SixDrepNet, FaceDetectionNode

def main():
    # Define the node name and software version
    node_name = "faceDetection"
    software_version = " v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
        "\t\t\t    Website: www.cssr4africa.org\n"
        "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
    )
    rospy.init_node(node_name)
    
    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")
    
    # Read the configuration file
    config = FaceDetectionNode.read_json_file('cssr_system')
    
    unit_tests = rospy.get_param('/faceDetection/unit_tests', default=False)
    
    if not unit_tests:
        rospy.set_param('/faceDetection_config', config)
    else:
        # Create a filtered config without the excluded keys
        filtered_config = {k: v for k, v in config.items() 
                        if k not in ["useCompressed", "algorithm", "verboseMode"]}
        
        # Set the filtered parameters to the parameter server
        for key, value in filtered_config.items():
            rospy.set_param('/faceDetection_config/' + key, value)

        # Set the algorithm, useCompressed, and verboseMode parameters
        config_test = FaceDetectionNode.read_json_file('unit_tests')
        
        # Filter and set only the specific parameters from the test config
        for key, value in config_test.items():
            if key in ["useCompressed", "algorithm", "verboseMode"]:
                rospy.set_param('/faceDetection_config/' + key, value)
        
    algorithm = rospy.get_param('/faceDetection_config/algorithm', default="sixdrep")

    if algorithm == 'mediapipe':
        face_detection = MediaPipe()
        face_detection.spin()
    elif algorithm == 'sixdrep':
        face_detection = SixDrepNet()
        face_detection.spin()
    else:
        rospy.logerr("Invalid algorithm selected. Exiting...")
        rospy.signal_shutdown("Invalid algorithm selected")

if __name__ == '__main__':
    main()