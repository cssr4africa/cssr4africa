"""
speech_event_test_class.py - unittest class implementing speechEvent test cases

Author:     Clifford Onyonka
Date:       2025-07-11
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import subprocess
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


class ROSNodeNotFoundError(FileNotFoundError):
    pass


class TestSpeechEvent(unittest.TestCase):
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

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )

        self.assertEqual(TestSpeechEvent._set_enabled("agree"), False)
        test_results = "Set unsupported status (agree): PASS"

        self.assertEqual(TestSpeechEvent._set_enabled("1"), False)
        test_results += "\nSet unsupported status (1): PASS"

        self.assertEqual(TestSpeechEvent._set_enabled("false"), True)
        test_results += "\nSet supported status (false): PASS"

        self.assertEqual(TestSpeechEvent._set_enabled("true"), True)
        test_results += "\nSet supported status (true): PASS"

    def test_set_language(self):
        """
        SpeechEvent test set language
        """
        global test_results

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )

        self.assertEqual(TestSpeechEvent._set_language("Kiswahili"), False)
        test_results = "Set unsupported language (Kiswahili): PASS"

        self.assertEqual(TestSpeechEvent._set_language("Arabic"), False)
        test_results += "\nSet unsupported language (Arabic): PASS"

        self.assertEqual(TestSpeechEvent._set_language("English"), True)
        test_results += "\nSet supported language (English): PASS"

        self.assertEqual(TestSpeechEvent._set_language("Kinyarwanda"), True)
        test_results += "\nSet supported language (Kinyarwanda): PASS"

    def test_transcription__mode_file(self):
        """
        SpeechEvent test transcription (mode: file)
        """
        global test_results

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not TestSpeechEvent._is_speech_event_driver_running():
            raise ROSNodeNotFoundError(
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
        expected_transcription = TestSpeechEvent._set_test_file(cursor)
        TestSpeechEvent._set_language("Kinyarwanda")
        while expected_transcription is not None:
            if expected_transcription.split("-")[0] != "rw":
                cursor += 1
                expected_transcription = TestSpeechEvent._set_test_file(cursor)
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
            expected_transcription = TestSpeechEvent._set_test_file(cursor)

        cursor = 0
        expected_transcription = TestSpeechEvent._set_test_file(cursor)
        TestSpeechEvent._set_language("English")
        while expected_transcription is not None:
            if expected_transcription.split("-")[0] != "en":
                cursor += 1
                expected_transcription = TestSpeechEvent._set_test_file(cursor)
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
            expected_transcription = TestSpeechEvent._set_test_file(cursor)

        display_process.terminate()
        display_process.wait()

    def test_transcription__mode_mic(self):
        """
        SpeechEvent test transcription (mode: mic)
        """
        global test_results

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_TOPIC} not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not TestSpeechEvent._is_speech_event_driver_running():
            raise ROSNodeNotFoundError(
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

        TestSpeechEvent._set_language("Kinyarwanda")
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
