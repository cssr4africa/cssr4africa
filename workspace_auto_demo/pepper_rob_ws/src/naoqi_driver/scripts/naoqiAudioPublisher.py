#!/usr/bin/env python3

import socket
import rospy
import numpy as np
from naoqi_driver.msg import AudioCustomMsg

def udp_ros_client():
    rospy.init_node('NaoqiAudioPublisher')
    pub = rospy.Publisher('/naoqi_driver/audio', AudioCustomMsg, queue_size=10)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', 9999))
    udp_socket.settimeout(1.0)  # Set a timeout of 1 second

    nbOfSamplesByChannel = 4096

    print("Naoqi Audio Publisher is running")

    try:
        while not rospy.is_shutdown():
            try:
                data, addr = udp_socket.recvfrom(32768)  # Adjust buffer size as needed
            except socket.timeout:
                continue  # No data, go back to the start of the loop

            audio = np.frombuffer(data, dtype=np.int16)

            rearLeft    = audio[0:nbOfSamplesByChannel].tolist()
            rearRight   = audio[nbOfSamplesByChannel:nbOfSamplesByChannel*2].tolist()
            frontLeft   = audio[nbOfSamplesByChannel*2:nbOfSamplesByChannel*3].tolist()
            frontRight  = audio[nbOfSamplesByChannel*3:nbOfSamplesByChannel*4].tolist()

            msg = AudioCustomMsg()
            msg.rearLeft = rearLeft
            msg.rearRight = rearRight
            msg.frontLeft = frontLeft
            msg.frontRight = frontRight
            pub.publish(msg)
            
    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt.")
    finally:
        udp_socket.close()
        print("UDP socket closed.")

if __name__ == "__main__":
    udp_ros_client()
