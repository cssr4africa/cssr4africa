"""
speech_event_test_implementation.py - functions to be used by the test ROS node

Author:     Clifford Onyonka
Date:       2025-02-23
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import subprocess
import sys
import threading
import time
import unittest

import rospy

from cssr_system.srv import set_enabled, set_language
from unit_tests.srv import set_next_test_file

SPEECH_TRANSCRIPTIONS_TOPIC = "/speechEvent/text"
SET_ENABLED_SERVICE = "/speechEvent/set_enabled"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SET_NEXT_TEST_FILE_SERVICE = "/speechEventDriver/set_next_test_file"
SOUND_DETECTION_TOPIC = "/soundDetection/signal"
MODE = "file"

test_results = None


class _ROSNodeNotFoundError(FileNotFoundError):
    pass


class _TestSpeechEvent(unittest.TestCase):
    @staticmethod
    def _is_speech_event_running():
        speech_event_is_running = False
        for (topic, _) in rospy.get_published_topics():
            if topic.strip() == SPEECH_TRANSCRIPTIONS_TOPIC.strip():
                speech_event_is_running = True
                break
        return speech_event_is_running

    @staticmethod
    def _is_speech_event_driver_running():
        speech_event_driver_is_running = False
        for (topic, _) in rospy.get_published_topics():
            if topic.strip() == SOUND_DETECTION_TOPIC.strip():
                speech_event_driver_is_running = True
                break
        return speech_event_driver_is_running

    @staticmethod
    def _set_test_file(cursor):
        try:
            response = rospy.ServiceProxy(
                SET_NEXT_TEST_FILE_SERVICE, set_next_test_file
            )(cursor)
            transcription = response.transcription if response.response == 1 else None
        except rospy.ServiceException as e:
            transcription = None
        return transcription

    @staticmethod
    def _set_enabled(status):
        try:
            response = rospy.ServiceProxy(
                SET_ENABLED_SERVICE, set_enabled
            )(status)
            success = True if response.response == 1 else False
        except rospy.ServiceException as e:
            success = False
        return success

    @staticmethod
    def _set_language(language):
        try:
            response = rospy.ServiceProxy(
                SET_LANGUAGE_SERVICE, set_language
            )(language)
            success = True if response.response == 1 else False
        except rospy.ServiceException as e:
            success = False
        return success

    def test_set_enabled(self):
        """
        SpeechEvent test set enabled
        """
        global test_results

        if not _TestSpeechEvent._is_speech_event_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )

        self.assertEqual(_TestSpeechEvent._set_enabled("agree"), False)
        test_results = "Set unsupported status (agree): PASS"

        self.assertEqual(_TestSpeechEvent._set_enabled("1"), False)
        test_results += "\nSet unsupported status (1): PASS"

        self.assertEqual(_TestSpeechEvent._set_enabled("false"), True)
        test_results += "\nSet supported status (false): PASS"

        self.assertEqual(_TestSpeechEvent._set_enabled("true"), True)
        test_results += "\nSet supported status (true): PASS"

    def test_set_language(self):
        """
        SpeechEvent test set language
        """
        global test_results

        if not _TestSpeechEvent._is_speech_event_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )

        self.assertEqual(_TestSpeechEvent._set_language("Kiswahili"), False)
        test_results = "Set unsupported language (Kiswahili): PASS"

        self.assertEqual(_TestSpeechEvent._set_language("Arabic"), False)
        test_results += "\nSet unsupported language (Arabic): PASS"

        self.assertEqual(_TestSpeechEvent._set_language("English"), True)
        test_results += "\nSet supported language (English): PASS"

        self.assertEqual(_TestSpeechEvent._set_language("Kinyarwanda"), True)
        test_results += "\nSet supported language (Kinyarwanda): PASS"

    def test_transcription__mode_file(self):
        """
        SpeechEvent test transcription (mode: file)
        """
        global test_results

        if not _TestSpeechEvent._is_speech_event_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not _TestSpeechEvent._is_speech_event_driver_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SOUND_DETECTION_TOPIC} not found, probably because "
                "no speechEventDriver ROS node is running"
            )

        display_process = subprocess.Popen(
            ["rostopic", "echo", SPEECH_TRANSCRIPTIONS_TOPIC],
            stdout=subprocess.PIPE
        )
        test_results = ""
        nl = "\n"

        cursor = 0
        expected_transcription = _TestSpeechEvent._set_test_file(cursor)
        _TestSpeechEvent._set_language("Kinyarwanda")
        while expected_transcription is not None:
            if expected_transcription.split("-")[0] != "rw":
                cursor += 1
                expected_transcription = _TestSpeechEvent._set_test_file(cursor)
                continue
            time.sleep(1)  # wait for set_next_test_file to take effect
            for i in range(3):
                display_process.stdout.readline()  # clear stdout
            line = display_process.stdout.readline().decode("UTF-8").strip()
            while "---" in line:
                line = display_process.stdout.readline().decode("UTF-8").strip()
            self.assertTrue(expected_transcription.split("-")[1] in line.lower())
            test_results += f"{'' if cursor == 0 else nl}Transcribe '{expected_transcription}': PASS"
            cursor += 1
            expected_transcription = _TestSpeechEvent._set_test_file(cursor)

        cursor = 0
        expected_transcription = _TestSpeechEvent._set_test_file(cursor)
        _TestSpeechEvent._set_language("English")
        while expected_transcription is not None:
            if expected_transcription.split("-")[0] != "en":
                cursor += 1
                expected_transcription = _TestSpeechEvent._set_test_file(cursor)
                continue
            time.sleep(1)  # wait for set_next_test_file to take effect
            for i in range(3):
                display_process.stdout.readline()  # clear stdout
            line = display_process.stdout.readline().decode("UTF-8").strip()
            while "---" in line:
                line = display_process.stdout.readline().decode("UTF-8").strip()
            self.assertTrue(expected_transcription.split("-")[1] in line.lower())
            test_results += f"{nl}Transcribe '{expected_transcription}': PASS"
            cursor += 1
            expected_transcription = _TestSpeechEvent._set_test_file(cursor)

        display_process.terminate()
        display_process.wait()

    def test_transcription__mode_mic(self):
        """
        SpeechEvent test transcription (mode: mic)
        """
        global test_results

        if not _TestSpeechEvent._is_speech_event_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not _TestSpeechEvent._is_speech_event_driver_running():
            raise _ROSNodeNotFoundError(
                f"ROS topic {SOUND_DETECTION_TOPIC} not found, probably because "
                "no speechEventDriver ROS node or soundDetection ROS node is running"
            )

        display_process = subprocess.Popen(
            ["rostopic", "echo", SPEECH_TRANSCRIPTIONS_TOPIC],
            stdout=subprocess.PIPE
        )
        test_results = ""
        is_time_over = False

        def transcribe():
            global test_results
            while not is_time_over:
                line = display_process.stdout.readline().decode("UTF-8").strip()
                while "---" in line:
                    line = display_process.stdout.readline().decode("UTF-8").strip()
                print(f"speechEventTest: transcribed -> {line[6:]}")
                test_results += f"{line[6:]}\n"

        _TestSpeechEvent._set_language("Kinyarwanda")
        t = threading.Thread(target=transcribe)
        t.start()
        print(
            "speechEventTest: within 60 seconds, read aloud the test utterances, "
            "pausing for about 3 seconds between each utterance"
        )
        time.sleep(60)
        is_time_over = True
        t.join()

        display_process.terminate()
        display_process.wait()


def _test_speech_event_helper(method_name, test_report):
    result = _TestSpeechEvent(method_name).run()

    if result.wasSuccessful():
        report = test_report.format(result=test_results)
    elif len(result.errors) > 0:
        report = test_report.format(result="ERR")
        report += "\r" + result.errors[0][1] + "\n"
    else:
        report = test_report.format(result="FAIL")
        report += "\r" + result.failures[0][1] + "\n"

    return report


def test_speech_event__set_enabled():
    test_report = "Test SpeechEvent's /speechEvent/set_enabled ROS service\n" \
        "---\n" \
        "As a:     behaviourController developer\n" \
        "I want:   a means of enabling and disabling the transcription process at runtime\n" \
        "So that:  I can stop the robot from transcribing its own speech by disabling the transcription process\n" \
        "---\n" \
        "{result}\n"
    return _test_speech_event_helper("test_set_enabled", test_report)


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
        "When:     an audio signal is detected on the /soundDetection/signal ROS topic\n" \
        "Then:     the audio needs to be transcribed and the text transcription published on the /speechEvent/text ROS topic\n" \
        "---\n" \
        "{result}\n"
    if MODE == "file":
        return _test_speech_event_helper("test_transcription__mode_file", test_report)
    else:
        return _test_speech_event_helper("test_transcription__mode_mic", test_report)


def parse_config_file(config_file_path):
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


def initialise(topic, mode):
    """ Make preparatory initialisations before running speechEvent tests

    Patameters:
        topic:  the speechEvent transcription topic
        mode:   whether speechEvent is transcribing from saved files or real-time
            human speech into the PC's or Pepper's microphones
    """
    global SPEECH_TRANSCRIPTIONS_TOPIC, MODE

    SPEECH_TRANSCRIPTIONS_TOPIC = topic
    MODE = mode

    if MODE.lower() not in ["mic", "file"]:
        rospy.logerr(f"speechEventTest: mode accepts either 'mic' of 'file', but '{MODE}' given")
        sys.exit(1)
