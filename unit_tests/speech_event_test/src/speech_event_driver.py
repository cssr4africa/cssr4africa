#!/usr/bin/env python3

"""
speech_event_driver.py - program that emulates a soundDetection ROS node

Author:     Clifford Onyonka
Date:       2025-02-23
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
speech_event_driver.py - program that emulates a soundDetection ROS node

This program's main purpose is helping in testing the speechEvent ROS node.
In PC mode, it captures audio from a computer's microphone and publishes it
on the /soundDetection/signal ROS topic, mimicking the functionality
provided by the soundDetection ROS node. In testing mode, it mimics
soundDetection by reading an audio file and publishing the audio signal to
the /soundDetection/signal ROS topic periodically.

Libraries:
- Ubuntu libraries: portaudio19-dev, python3-dev, python3-pyaudio
- Python libraries: numpy, pyaudio, rospy, scipy, std_msgs

Parameters:
- None

Command-line Parameters:
- None

Configuration File Parameters:
- channels                              1
- chunkSize                             1024
- sampleRate                            48000
- speechAmplitudeThreshold              0.25
- mode                                  mic

Subscribed Topics and Message Types:
- None

Published Topics and Message Types:
- /soundDetection/signal                std_msgs/Float32MultiArray

Services Invoked:
- None

Services Advertised and Request Message:
- /speechEventTest/set_next_test_file   <int>

Input Data Files:
- *.wav (audio test files)

Output Data Files:
- None

Configuration Files:
- speech_event_driver.ini

Example Instantiation of the Module:
- rosrun unit_tests speech_event_driver.py

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2025-02-23
Version:    v1.0
"""

import os
import queue
import sys
import time

import numpy as np
import pyaudio
import rospy
from scipy.io import wavfile
from std_msgs.msg import Float32MultiArray

from unit_tests.srv import set_next_test_file, set_next_test_fileResponse


# Static config options (not set via config file)
NODE_NAME = "speechEventDriver"
PUB_TOPIC = "/soundDetection/signal"
FORMAT = pyaudio.paFloat32
ROS_LOGGER_THROTTLE_SPEECH_DETECTED = 1
SET_NEXT_TEST_FILE_SERVICE = "/speechEventDriver/set_next_test_file"

# Config options set via config file
CHANNELS = 1
CHUNK_SIZE = 1024
SAMPLE_RATE = 48000
SPEECH_AMPLITUDE_THRESHOLD = 0.125  # amplitude below which a signal is assumed to be ambient noise
MODE = "mic"  # whether to use audio from PC's microphones or from saved audio files

buffer_size = SAMPLE_RATE // CHUNK_SIZE
testing_samples = None
test_files = {}  # {cursor_position: [transcription_text, audio_file_path], ...}


def _parse_config_file(config_file_path):
    """ Get a dict representing the configuration options stored in the
    passed configuration file

    Parameters:
        config_file_path (str): path to a configuration file

    Returns:
        dict:    key-value pairs of configurations stored in the passed
            config file
    """
    config = {}

    with open(config_file_path, "r") as f:
        for line in f.readlines():
            if len(line) < 1:
                continue
            a_list = line.split("\t") if "\t" in line else line.split(" ")
            config[a_list[0]] = a_list[-1]
    
    return config


def _is_utterance_present(samples):
    """ Find out if a speech utterance may be contained in the passed audio
    signal samples (this function returns true even when only noise, and no real
    utterance, is contained in the signal)

    Parameters:
    samples (np.array): array of audio samples

    Returns:
    bool:    True if an utterance is found in the signal samples, else False
    """
    for i in samples:
        if np.abs(i) >= SPEECH_AMPLITUDE_THRESHOLD:
            return True

    return False


def _run_mic_mode_helper(stream, publisher):
    """ Main function that runs the sound capture and publishing routine

    Parameters:
    stream (PyAudio.Stream):        PyAudio Stream object for reading audio
        captured by a microphone
    publisher (rospy.Publisher):    a ROS publisher object to publish the
        captured audio to a ROS topic
    """
    start_pad_samples = queue.Queue(maxsize=buffer_size)
    stop_pad_samples_count = 0
    rate = rospy.Rate(500)

    while not rospy.is_shutdown():
        incoming_samples = np.frombuffer(stream.read(CHUNK_SIZE), dtype=np.float32)
        is_utterance = _is_utterance_present(incoming_samples)

        if not is_utterance:
            if stop_pad_samples_count == 0:
                try:
                    start_pad_samples.put(incoming_samples, block=False)
                except queue.Full:
                    start_pad_samples.get()
                    start_pad_samples.put(incoming_samples, block=False)
                continue

        if is_utterance:
            stop_pad_samples_count = buffer_size
        else:
            stop_pad_samples_count -= 1

        rospy.loginfo_throttle(
            ROS_LOGGER_THROTTLE_SPEECH_DETECTED, "speechEventDriver: speech detected"
        )

        while True:
            try:
                rate.sleep()  # Appears useless, but if deleted then the next line won't work
                publisher.publish(Float32MultiArray(data=start_pad_samples.get(block=False)))
            except queue.Empty:
                break

        publisher.publish(Float32MultiArray(data=incoming_samples))


def _set_next_test_file_handler(req):
    """
    Begin publishing audio signals from the next audio file in the list everytime
    the set_next_file service gets called

    Parameters:
        req:    the service's request object
    """
    global testing_samples

    if req.cursor not in test_files:
        return set_next_test_fileResponse(response=0, transcription="<No more test files>")

    _, testing_samples = wavfile.read(test_files[req.cursor][1])

    return set_next_test_fileResponse(response=1, transcription=test_files[req.cursor][0])


def run_mic_mode():
    """
    Publish audio streamed from the PC's microphones
    """
    py_audio = pyaudio.PyAudio()
    stream = py_audio.open(
        rate=SAMPLE_RATE, channels=CHANNELS, format=FORMAT, input=True,
        frames_per_buffer=CHUNK_SIZE
    )
    publisher = rospy.Publisher(PUB_TOPIC, Float32MultiArray, queue_size=3)

    print(
        """
        \rspeechEventDriver: Tweak the speechAmplitudeThreshold in case the driver is having trouble capturing speech utterances.
        \rIncrease it in case speech is not being detected, and decrease it in case ambient noise is being captured together with
        \rthe speech utterances. You may also tweak the sensitivity of the PC's microphones in case speechAmplitudeThreshold fails
        \rto solve the aforementioned issues that may arise.
        """
    )
    rospy.loginfo(f"speechEventDriver: {NODE_NAME} is running")

    try:
        _run_mic_mode_helper(stream, publisher)
    except rospy.ROSInterruptException:
        pass

    stream.stop_stream()
    stream.close()
    py_audio.terminate()


def run_file_mode():
    """
    Publish audio read from a saved file
    """
    publisher = rospy.Publisher(PUB_TOPIC, Float32MultiArray, queue_size=3)

    rospy.loginfo("speechEventDriver: ROS node '%s' is running ..." % NODE_NAME)
    rospy.Service(
        SET_NEXT_TEST_FILE_SERVICE, set_next_test_file, _set_next_test_file_handler
    )
    rospy.loginfo("speechEventDriver: {NODE_NAME} is running")

    try:
        while not rospy.is_shutdown():
            publisher.publish(Float32MultiArray(data=testing_samples))
            time.sleep(3.5)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    current_file_path = os.path.abspath(__file__)
    current_file_dir = os.path.dirname(current_file_path)
    config_file_path = os.path.join(
        os.path.dirname(current_file_dir), "config", "speech_event_driver_configuration.ini"
    )
    test_audio_files = [
        (
            i.split(".")[0].replace("_", " "),
            os.path.join(os.path.dirname(current_file_dir), "data", i)
        ) for i in os.listdir(os.path.join(os.path.dirname(current_file_dir), "data"))
        if i.split(".")[1] == "wav"
    ]

    config = _parse_config_file(config_file_path)

    CHANNELS = int(config["channels"].strip())
    CHUNK_SIZE = int(config["chunkSize"].strip())
    SAMPLE_RATE = int(config["sampleRate"].strip())
    SPEECH_AMPLITUDE_THRESHOLD = float(config["speechAmplitudeThreshold"].strip())
    MODE = config["mode"].strip().lower()

    buffer_size = (SAMPLE_RATE // CHUNK_SIZE) * 2
    test_files = {idx: item for idx, item in enumerate(test_audio_files)}
    _, testing_samples = wavfile.read(test_files[0][1])

    rospy.init_node(NODE_NAME, anonymous=True)

    MODE = rospy.get_param("/speechEvent/param_mode", default=MODE).strip().lower()

    if MODE not in ["mic", "file"]:
        rospy.logerr(f"speechEventDriver: mode accepts either 'mic' of 'file', but '{MODE}' given")
        sys.exit(1)

    run_mic_mode() if MODE == "mic" else run_file_mode()
