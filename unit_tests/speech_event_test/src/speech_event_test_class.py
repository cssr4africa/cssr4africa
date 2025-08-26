"""
speech_event_test_class.py - unittest class implementing speechEvent test cases

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

import threading
import time
import unittest

import actionlib
import rospy

from cssr_system.msg import recognise_speechAction, recognise_speechGoal, recognise_speechResult, recognise_speechFeedback
from cssr_system.srv import set_language
from unit_tests.srv import set_next_test_file


SPEECH_TRANSCRIPTIONS_ACTION = "/speechEvent/recognise_speech_action"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SET_NEXT_TEST_FILE_SERVICE = "/speechEventDriver/set_next_test_file"
SOUND_DETECTION_TOPIC = "/soundDetection/signal"

test_results = None
client = actionlib.SimpleActionClient(SPEECH_TRANSCRIPTIONS_ACTION, recognise_speechAction)
goal = recognise_speechGoal()
goal.wait = 5.0
timeout = rospy.Duration(10.0)

print(f"Waiting for {SPEECH_TRANSCRIPTIONS_ACTION} server ...")
client.wait_for_server()
print(f"Acquired {SPEECH_TRANSCRIPTIONS_ACTION} server")


class ROSNodeNotFoundError(FileNotFoundError):
    pass


class TestSpeechEvent(unittest.TestCase):
    @staticmethod
    def _is_speech_event_running():
        speech_event_is_running = False
        for (topic, _) in rospy.get_published_topics():
            if topic.strip() == f"{SPEECH_TRANSCRIPTIONS_ACTION}/goal".strip():
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
    def _set_language(language):
        try:
            response = rospy.ServiceProxy(
                SET_LANGUAGE_SERVICE, set_language
            )(language)
            success = True if response.response == 1 else False
        except rospy.ServiceException as e:
            success = False
        return success

    def test_set_language(self):
        """
        SpeechEvent test set language
        """
        global test_results

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_ACTION}/goal not found, "
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
                f"ROS topic {SPEECH_TRANSCRIPTIONS_ACTION}/goal not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not TestSpeechEvent._is_speech_event_driver_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SOUND_DETECTION_TOPIC} not found, probably because "
                "no speechEventDriver ROS node is running"
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
            client.send_goal(goal)
            client.wait_for_result(timeout)
            self.assertTrue(expected_transcription.split("-")[1] in client.get_result().transcription.lower())
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
            client.send_goal(goal)
            client.wait_for_result(timeout)
            self.assertTrue(expected_transcription.split("-")[1] in client.get_result().transcription.lower())
            test_results += f"{nl}Transcribe '{expected_transcription}': PASS"
            cursor += 1
            expected_transcription = TestSpeechEvent._set_test_file(cursor)

    def test_transcription__mode_mic(self):
        """
        SpeechEvent test transcription (mode: mic)
        """
        global test_results

        if not TestSpeechEvent._is_speech_event_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SPEECH_TRANSCRIPTIONS_ACTION}/goal not found, "
                "probably because no speechEvent ROS node is running"
            )
        if not TestSpeechEvent._is_speech_event_driver_running():
            raise ROSNodeNotFoundError(
                f"ROS topic {SOUND_DETECTION_TOPIC} not found, probably because "
                "no speechEventDriver ROS node or soundDetection ROS node is running"
            )

        test_results = ""
        is_time_over = False

        def transcribe():
            global test_results
            while not is_time_over:
                client.send_goal(goal)
                client.wait_for_result(timeout)
                result = client.get_result().transcription
                print(f"speechEventTest: transcribed -> {result.lower()}")
                test_results += f"{result.lower()}\n"

        TestSpeechEvent._set_language("Kinyarwanda")
        t = threading.Thread(target=transcribe)
        t.start()
        print(
            "speechEventTest: read aloud Kinyarwanda test utterances within 30 seconds, "
            "pausing for about 3 seconds between each utterance"
        )
        time.sleep(30)
        is_time_over = True
        t.join()

        TestSpeechEvent._set_language("English")
        t = threading.Thread(target=transcribe)
        t.start()
        print(
            "speechEventTest: read aloud English test utterances within 30 seconds, "
            "pausing for about 3 seconds between each utterance"
        )
        time.sleep(30)
        is_time_over = True
        t.join()
