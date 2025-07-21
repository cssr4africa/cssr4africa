#!/usr/bin/python2

""" 
send_and_play_audio.py - functionality to send and play audio on the robot for the transcribed kinyarwanda text

Author:     Muhirwa Richard
Date:       2025-04-18
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import sys
import os

# Check if verbose mode is enabled from the environment variable
verbose_mode = os.environ.get('VERBOSE_AUDIO', '0') == '1'

# Set environment variables and redirect streams BEFORE importing naoqi
if not verbose_mode:
    # Suppress NAOqi logs only in non-verbose mode
    os.environ['VERBOSE'] = '0'
    os.environ['QI_LOG_LEVEL'] = 'silent'
    os.environ['QI_LOG_CONTEXT'] = '0'
    os.environ['QI_LOG_SYNCHRONOUS'] = '0'
    
    # Redirect stdout and stderr to suppress all output
    sys.stdout = open(os.devnull, 'w')
    sys.stderr = open(os.devnull, 'w')

# Now import naoqi (this will be silent if verbose_mode is False)
from naoqi import ALProxy
import subprocess

def main():
    try:
        audio_path = sys.argv[1]
        audio_name = audio_path.split("/")[-1]
        IP = sys.argv[2]
        PORT = int(sys.argv[3])
        
        if verbose_mode:
            print("Initializing the audio player")
        
        # Copy file to robot
        subprocess.call([
            "sshpass", "-p", "nao", "scp", "-o", "StrictHostKeyChecking=no", 
            audio_path, "nao@{0}:/home/nao".format(IP)
        ])
        
        # Create audio player proxy
        audio_player = ALProxy("ALAudioPlayer", IP, PORT)
        
        # Play the audio file
        audio_player.playFile("/home/nao/" + audio_name)
        
        if verbose_mode:
            print("After playing the audio")
        
        # Clean up the file from robot
        subprocess.call([
            "sshpass", "-p", "nao", "ssh", "-o", "StrictHostKeyChecking=no", 
            "nao@{0}".format(IP), "rm /home/nao/" + audio_name
        ])
        
        if verbose_mode:
            print("Audio file cleaned up from robot")
        
    except Exception as e:
        # Only show errors in verbose mode
        if verbose_mode:
            print("Error: {0}".format(e))
        sys.exit(1)

if __name__ == "__main__":
    main()
