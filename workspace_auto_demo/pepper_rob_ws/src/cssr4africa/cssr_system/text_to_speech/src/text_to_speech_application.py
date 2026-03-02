#!/usr/bin/env python3

""" 
text_to_speech_application.py - ROS node for integrated multilingual text-to-speech functionality

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec) 
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.

"""

"""
> text_to_speech_application.py - ROS node for integrated multilingual text-to-speech functionality

This ROS application node provides text-to-speech (TTS) services supporting both Kinyarwanda and English languages.
The node integrates with Pepper robot's audio system for English TTS and uses a custom TTS model for Kinyarwanda.
It exposes a ROS service interface that accepts text messages and language specifications, then generates and plays
the corresponding speech audio. 

> Libraries
    - rospy
    - os
    - sys
    - threading
    - time
    - text_to_speech_implementation
    - cssr_system.srv: (TTS, TTSResponse)

> Parameters
    > Command line parameters:
        None

    > Configuration File Parameters
        - language: (english/kinyarwanda)
        - verboseMode: (True/False)
        - ip: Network IP address of the Pepper robot
        - port: Communication port for robot connection
        - useCuda: (True/False)

> Subscribed Topics 
    - None 

> Published Topics and Message Types
    - /speech (std_msgs/String): English text messages for Pepper robot's built-in TTS

> Services Invoked
    - None 

> Services Advertised and Request Message
    - /textToSpeech/say_text (cssr_system/TTS)
      Request: {string message, string language}
      Response: {bool success}
  

> Input Data Files
    - None

> Output Data Files
    - Temporary .wav files: Generated audio files for Kinyarwanda speech (auto-deleted)
    - ROS log files: Node operation logs in ~/.ros/log/

> Configuration Files
    - text_to_speech_configuration.ini

> Example Instantiation of the Module
    - rosrun cssr_system text_to_speech_application.py
    - roslaunch cssr_system text_to_speech_launch_robot.launch

- Author:  Muhirwa Richard, Carnegie Mellon University Africa
- Email:   muhirwarichard1@gmail.com
- Date:    2025-04-18
- Version: v1.0
"""

import rospy
from cssr_system.srv import TTS, TTSResponse
import os
import sys

# Create __init__.py if it doesn't exist
# Without __init__.py imports will fail in Python < 3.3

current_file_dir = os.path.dirname(__file__)
init_file = os.path.join(current_file_dir, '__init__.py')
if not os.path.exists(init_file):
    with open(init_file, 'w') as f:
        f.write('# This file makes the directory a Python package\n')

# Add the current directory to Python path
sys.path.insert(0, current_file_dir)

# Import the implementation
from text_to_speech_implementation import TTSImplementation

software_version = "version v1.0"

def handle_say_text(request, tts_impl):
    """Handle the say_text service request
    
    Args:
        request: The ROS service request containing message and language
        tts_impl: The TTS implementation instance
        
    Returns:
        TTSResponse: Response indicating success or failure
    """
    success = tts_impl.say_text(request.message, request.language.lower())
    return TTSResponse(success=success)

def text_to_speech_node():
    
    # Initialize the ROS node
    rospy.init_node('textToSpeech')
    
    tts_impl = None
    
    try:
        # Create an instance of the TTS implementation
        tts_impl = TTSImplementation()
        
        # Register the service with ROS
        rospy.Service(
            "/textToSpeech/say_text", 
            TTS, 
            lambda req: handle_say_text(req, tts_impl)
        )
        
        
        copyright_message = (
            f"textToSpeech  {software_version}\n"
            "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
            "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
            "\t\t\t    Website: www.cssr4africa.org\n"
            "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
         )
        
        rospy.loginfo("textToSpeech: startup.")
        rospy.loginfo("textToSpeech: publish to /speech.")
        rospy.loginfo("textToSpeech: /textToSpeech/say_text service advertised")
        
        # Construct the copyright message
        rospy.loginfo(copyright_message)
        # Keep the node running until shutdown
        rospy.spin()
        
    except Exception as e:
        rospy.logerr(f"Error initializing text-to-speech node: {e}")
        exit(1)

if __name__ == '__main__':
    # Set up dynamic paths
    current_file_dir = os.path.dirname(__file__)
    parent_dir = os.path.dirname(current_file_dir)
    
    config_file_path = os.path.join(parent_dir, "config", "text_to_speech_configuration.ini")
    model_dir_path = os.path.join(parent_dir, "model")
    python2_script_path = os.path.join(current_file_dir, "send_and_play_audio.py")
    
    # Pass paths to implementation
    TTSImplementation.set_paths(
        config_file_path=config_file_path,
        model_dir_path=model_dir_path,
        python2_script_path=python2_script_path
    )
    
    text_to_speech_node()