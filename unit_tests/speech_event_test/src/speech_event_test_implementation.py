"""
speech_event_test_implementation.py - functions to be used by the test ROS node

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2025-02-23
Version:    v1.0

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2026-04-21
Version:    v1.1

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import sys

import rospy

import speech_event_test_class as se_test_class

MODE = "file"


def _test_speech_event_helper(method_name, test_report):
    result = se_test_class.TestSpeechEvent(method_name).run()

    if result.wasSuccessful():
        report = test_report.format(result=se_test_class.test_results)
    elif len(result.errors) > 0:
        report = test_report.format(result="ERR")
        report += "\r" + result.errors[0][1] + "\n"
    else:
        report = test_report.format(result="FAIL")
        report += "\r" + result.failures[0][1] + "\n"

    return report


def test_speech_event__set_language():
    test_report = "Test SpeechEvent's /speechEvent/set_language ROS service\n" \
        "---\n" \
        "As a:     behaviourController developer\n" \
        "I want:   a means of setting the transcription language of speechEvent at runtime\n" \
        "So that:  I don't have to restart speechEvent every time I update the transcription language\n" \
        "---\n" \
        "{result}\n"
    return _test_speech_event_helper("test_set_language", test_report)


def test_speech_event__transcription():
    test_report = "Test SpeechEvent's end-to-end transcription process\n" \
        "---\n" \
        "Given:    a running speechEvent ROS node\n" \
        "When:     a goal is sent to the /speechEvent/recognise_speech_action ROS action\n" \
        "Then:     transcribe utterances in audio published on the /soundDetection/signal ROS topic and return the result via the /speechEvent/recognise_speech_action ROS action\n" \
        "---\n" \
        "{result}\n"
    if MODE == "file":
        return _test_speech_event_helper("test_transcription__mode_file", test_report)
    else:
        return _test_speech_event_helper("test_transcription__mode_mic", test_report)


def initialise(mode):
    """ Make preparatory initialisations before running speechEvent tests

    Patameters:
        mode:   whether speechEvent is transcribing from saved files or real-time
            human speech into the PC's or Pepper's microphones
    """
    global MODE

    MODE = mode

    if MODE.lower() not in ["mic", "file"]:
        rospy.logerr(f"speechEventTest: mode accepts either 'mic' of 'file', but '{MODE}' given")
        sys.exit(1)
