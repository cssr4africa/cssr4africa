#!/usr/bin/env python2

"""
The script subscribes to the ALAudioSource and get the raw audio buffer from the 
all four channels. It then sends the audio buffer through UDP over the local network 
to a client script that will publish the audio data to ROS Noetic.

The reason for this is that the NOAqiAudio script run on python2 and the NaoqiAudioPublisher 
script runs on python3 due to the fact that ROS Noetic is only compatible with python3. 
This script is a bridge between the two. 

Author: Yohannes Haile
Date: Jan 28, 2024
"""

import qi
import argparse
import sys
import time
import socket

class SoundProcessingModule(object):
    def __init__(self, app):
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session
        
        # Get the service ALAudioDevice.
        self.audio_service = session.service("ALAudioDevice")  # Initialize audio service
        self.isProcessingDone = False
        self.module_name = "SoundProcessingModule"
        self.audio = []

    def startProcessing(self):
        self.audio_service.setClientPreferences(self.module_name, 48000, 0, 1)
        self.audio_service.subscribe(self.module_name)

        while self.isProcessingDone == False:
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        udp_socket.sendto(inputBuffer, ('localhost', 9999))

if __name__ == "__main__":
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.29.111.230", help="Robot IP address.")
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
    args, _ = parser.parse_known_args()

    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + connection_url])
        MySoundProcessingModule = SoundProcessingModule(app)
        app.session.registerService("SoundProcessingModule", MySoundProcessingModule)
        MySoundProcessingModule.startProcessing()

    except KeyboardInterrupt:
    
        # stop audio processing
        app.isProcessingDone = True
        udp_socket.close()
        app.stop()
        sys.exit(0)

