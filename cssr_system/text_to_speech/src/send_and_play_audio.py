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


# Suppress NAOqi logs
os.environ['VERBOSE'] = '0'
os.environ['QI_LOG_LEVEL'] = 'silent'

# Redirect stdout and stderr if you want complete silence
if not os.environ.get('VERBOSE_AUDIO', '0') == '1':
    sys.stdout = open(os.devnull, 'w')
    sys.stderr = open(os.devnull, 'w')


from naoqi import ALProxy
import subprocess


audio_path = sys.argv[1]
audio_name = audio_path.split("/")[-1]
IP = sys.argv[2]
PORT = int(sys.argv[3])

subprocess.call(["sshpass","-p","nao","scp","-o","StrictHostKeyChecking=no",audio_path,"nao@{0}:/home/nao".format(IP)])
print ("Initializing the audio player")
audio_player = ALProxy("ALAudioPlayer",IP,PORT)

audio_player.playFile("/home/nao/"+audio_name)
print ("After playing the audio")
subprocess.call(["sshpass","-p","nao","ssh","-o","StrictHostKeyChecking=no","nao@{0}".format(IP),"rm /home/nao/"+audio_name])

