#!/usr/bin/env python3

"""
text_to_speech_application.py - ROS node for integrated multilingual text-to-speech

Author:     Muhirwa Richard
Date:       2026-04-27
Version:    v1.1

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
> text_to_speech_application.py - ROS node for multilingual TTS (Kinyarwanda / English)

Supports two languages:
  - Kinyarwanda : fine-tuned YourTTS model  (model/ directory)
  - English     : XTTS v2 model             (englishModelPath in config)

The communication interface and playback destination are controlled by the
configuration file:

  interface     = service | action | both
  playback_mode = naoqi   | local

> ROS interfaces (all at /textToSpeech/say_text)
  Service (cssr_system/TTS):
      Request:  { string message, string language }
      Response: { bool success }

  Action (cssr_system/TTSAction):
      Goal:     { string text, string language }
      Feedback: { string status, float32 progress }
      Result:   { bool success, string message, string audio_file_path }

> Published Topics
  None (audio is played directly, not published)

> Configuration File
  text_to_speech_configuration.ini

> Example
  rosrun cssr_system text_to_speech_application.py
  roslaunch cssr_system text_to_speech_launch_robot.launch
"""

import os
import sys

import rospy
import actionlib

current_file_dir = os.path.dirname(__file__)
init_file = os.path.join(current_file_dir, '__init__.py')
if not os.path.exists(init_file):
    with open(init_file, 'w') as f:
        f.write('# This file makes the directory a Python package\n')

sys.path.insert(0, current_file_dir)

from text_to_speech_implementation import TTSImplementation

from cssr_system.srv import TTS, TTSResponse
from cssr_system.msg import TTSAction, TTSFeedback, TTSResult

SOFTWARE_VERSION = "v2.0"
TTS_TOPIC = "/textToSpeech/say_text"


class TextToSpeechNode:

    def __init__(self, tts_impl):
        self.tts_impl = tts_impl
        self.interface = tts_impl.config.get('interface', 'service')

        if self.interface not in ('service', 'action', 'both'):
            rospy.logwarn(
                f"Unknown interface '{self.interface}'. "
                "Valid values: 'service', 'action', 'both'. Defaulting to 'service'."
            )
            self.interface = 'service'

        if self.interface in ('service', 'both'):
            self._start_service()

        if self.interface in ('action', 'both'):
            self._start_action_server()

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------

    def _start_service(self):
        rospy.Service(TTS_TOPIC, TTS, self._handle_service)
        rospy.loginfo(f"textToSpeech: service advertised at {TTS_TOPIC}")

    def _handle_service(self, req):
        lang = req.language.lower() if req.language else self.tts_impl.config['language']
        success, _, _ = self.tts_impl.say_text(req.message, lang)
        return TTSResponse(success=success)

    # ------------------------------------------------------------------
    # Action server
    # ------------------------------------------------------------------

    def _start_action_server(self):
        self._action_server = actionlib.SimpleActionServer(
            TTS_TOPIC,
            TTSAction,
            execute_cb=self._execute_action,
            auto_start=False,
        )
        self._action_server.start()
        rospy.loginfo(f"textToSpeech: action server started at {TTS_TOPIC}")

    def _execute_action(self, goal):
        text = goal.text
        lang = goal.language.lower() if goal.language else self.tts_impl.config['language']

        rospy.loginfo(f"[action] '{text}' [{lang}]")

        feedback = TTSFeedback()
        result   = TTSResult()

        try:
            feedback.status   = "Synthesizing"
            feedback.progress = 0.2
            self._action_server.publish_feedback(feedback)

            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                return

            audio_file = self.tts_impl.synthesize(text, lang)

            if self._action_server.is_preempt_requested():
                self._safe_remove(audio_file)
                self._action_server.set_preempted()
                return

            feedback.status   = "Playing audio"
            feedback.progress = 0.7
            self._action_server.publish_feedback(feedback)

            self.tts_impl.play_audio(audio_file)

            feedback.status   = "Completed"
            feedback.progress = 1.0
            self._action_server.publish_feedback(feedback)

            result.success         = True
            result.message         = f"Spoken '{text}' in {lang}"
            result.audio_file_path = audio_file
            self._action_server.set_succeeded(result)

        except Exception as exc:
            rospy.logerr(f"[action] error: {exc}")
            result.success         = False
            result.message         = str(exc)
            result.audio_file_path = ""
            self._action_server.set_aborted(result)

        finally:
            self._safe_remove(getattr(result, 'audio_file_path', ''))

    @staticmethod
    def _safe_remove(path):
        if path and os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:
                pass


# ---------------------------------------------------------------------------

def text_to_speech_node():
    rospy.init_node('textToSpeech')

    try:
        tts_impl = TTSImplementation()
        node = TextToSpeechNode(tts_impl)

        copyright_msg = (
            f"textToSpeech  {SOFTWARE_VERSION}\n"
            "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
            "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
            "\t\t\t    Website: www.cssr4africa.org\n"
            "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
        )
        rospy.loginfo("textToSpeech: startup.")
        rospy.loginfo(
            f"textToSpeech: interface={node.interface}  "
            f"language={tts_impl.config['language']}  "
            f"playback={tts_impl.config['playback_mode']}"
        )
        rospy.loginfo(copyright_msg)

        rospy.spin()

    except Exception as exc:
        rospy.logerr(f"Error initializing text-to-speech node: {exc}")
        sys.exit(1)


if __name__ == '__main__':
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir       = os.path.dirname(current_file_dir)

    TTSImplementation.set_paths(
        config_file_path   = os.path.join(parent_dir, "config", "text_to_speech_configuration.ini"),
        model_dir_path     = os.path.join(parent_dir, "model"),
        python2_script_path= os.path.join(current_file_dir, "send_and_play_audio.py"),
    )

    text_to_speech_node()
