#!/usr/bin/env python3

"""
sound_detection_test_application.py Application code to run the Sound Detection and Processing Unit test.

Author: Yohannes Tadesse Haile
Date: April 06, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
sound_detection_test_application.py  Application code to run the Sound Detection and Processing unit test.

This sound_detection_test is a unit test application code to test the sound detection algorithm.
It can record audio both from the filtered output and the original unfiltered microphone input,
generate plots of audio signals and sound direction data, and save them for analysis. The test
helps validate the correctness of audio signal processing and source localization.

Libraries
    - rospkg
    - rospy
    - os
    - json
    - numpy
    - soundfile
    - matplotlib
    - datetime
    - threading
    - std_msgs.msg (Float32MultiArray, Float32)
    - sound_detection.msg (sound_detection)

Parameters
    Launch File Parameters:
        roslaunch unit_tests sound_detection_test_launch.launch
            robot_ip: Pepper robot IP address (e.g 172.29.111.230 or 172.29.111.240)

    Configuration File Parameters
        Key                         Value
        saveDirectory               /path/to/save/directory
        sampleRate                  48000
        recordFiltered              true
        recordUnfiltered            true
        generatePlots               true
        recordDuration              10
        plotInterval                10
        plotDpi                     150
        maxDirectionPoints          100
        directionPlotYlimit         90
        verboseMode                 true

Subscribed Topics
    Topic Name                                  Message Type
    /naoqi_driver/audio                         sound_detection/sound_detection
    /soundDetection/signal                      std_msgs/Float32MultiArray
    /soundDetection/direction                   std_msgs/Float32

Published Topics
    None

Input Data Files
    - pepper_topics.dat: Data file for Pepper robot audio topics

Output Data Files
    - filtered_{timestamp}.wav: Filtered audio recordings
    - unfiltered_{timestamp}.wav: Unfiltered audio recordings
    - audio_signals_{timestamp}.png: Plot showing filtered and unfiltered signals
    - direction_data_{timestamp}.png: Plot showing sound direction over time

Configuration File
    sound_detection_test_configuration.json

Example of instantiation of the module
    roslaunch unit_tests sound_detection_test_launch.launch
"""

import rospy
from sound_detection_test_implementation import SoundDetectionTest

def main():
    # Define the node name and software version
    node_name = "soundDetectionTest"
    software_version = "v1.0"

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
    
    # Set the unit test flag to true for the main node to recognize
    rospy.set_param('/soundDetection/unit_tests', True)

    # Create the test instance
    # The configuration file is read inside the SoundDetectionTest class
    SoundDetectionTest()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == '__main__':
    main()