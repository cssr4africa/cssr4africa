#!/usr/bin/env python3

"""
sound_detection_application.py Application code to run the sound detection and localization algorithm.

Author: Yohannes Tadesse Haile
Date: April 13, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
sound_detection_application.py Application code to run the sound detection and localization algorithm.

The sound localization algorithm is implemented using a ROS audio topic that can be configured to receive audio from
a robot or an external microphone. It processes the incoming audio signal to detect sound events and localize the sound
source by computing the interaural time difference (ITD) using the GCC-PHAT method. The ITD is then converted into an
angle of arrival using physical parameters such as the speed of sound and the distance between the microphones.
This code contains the main function that initializes the sound localization node, loads the configuration, and starts
the algorithm. The algorithm is designed to accumulate fixed-size audio samples (e.g., 4096 per callback) in a rolling
buffer until sufficient data is collected for processing.

Libraries:
    - math
    - numpy
    - rospy
    - rospkg
    - os
    - json
    - webrtcvad
    - std_msgs
    - threading
    - noisereduce
    - soundfile
    - datetime

Parameters:
    Command line arguments: None

    Configuration File Parameters:
        Key                                      Value
        intenstiyThreshold                       [float]     e.g., 0.0039
        distanceBetweenEars                      [float]     e.g., 0.07
        localizationBufferSize                   [int]       e.g., 8192
        vadAggressiveness                        [int]       e.g., 1
        contextDuration                          [float]     e.g., 1.0
        useNoiseReduction                        [bool]      e.g., true
        verboseMode                              [bool]      e.g., true
        
Subscribed Topics:
    Topic Name                                   Message Type
    /naoqi_driver/audio                          sound_detection/sound_detection_microphone_msg_file.msg

Published Topics:
    Topic Name                                   Message Type
    /soundDetection/signal                       std_msgs/Float32MultiArray
    /soundDetection/direction                    std_msgs/Float32

Input Data Files:
    - pepper_topics.dat: Data file containing topic names for the robot's audio sources.

Output Data Files:
    None

Configuration File:
    sound_detection_configuration.json

Example of instantiation of the module:
    rosrun cssr_system sound_detection_application.py
"""

import rospy
from sound_detection_implementation import SoundDetectionNode 

def main():
    # Define the node name and software version
    node_name = "soundDetection"
    software_version = " v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
        "\t\t\t    Website: www.cssr4africa.org\n"
        "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
    )
    
    # Initialize the ROS node.
    rospy.init_node('soundDetection')

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")
    
    # Read the configuration file
    config = SoundDetectionNode.read_json_file('cssr_system')
    
    unit_tests = rospy.get_param('/soundDetection/unit_tests', default=False)
    
    if not unit_tests:
        # Use the standard configuration
        rospy.set_param('/soundDetection', config)
    else:
        # Unit test mode - load test configuration
        # Create a filtered config without the excluded keys (if needed)
        filtered_config = {k: v for k, v in config.items() 
                        if k not in ["verboseMode"]}
        
        # Set the filtered parameters to the parameter server
        for key, value in filtered_config.items():
            rospy.set_param('/soundDetection/' + key, value)

        # Set specific parameters from the test config
        config_test = SoundDetectionNode.read_json_file('unit_tests')
        
        # Filter and set only specific parameters from the test config
        for key, value in config_test.items():
            if key in ["verboseMode", "recordDuration"]:
                rospy.set_param('/soundDetection/' + key, value)
    
    # Create an instance of sound detection node
    node = SoundDetectionNode()
        
    # Run the node
    node.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass