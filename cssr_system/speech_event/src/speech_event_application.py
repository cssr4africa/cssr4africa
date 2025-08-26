#!/usr/bin/env python3

"""
speech_event_application.py - speechEvent ROS node definition

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
speech_event_application.py - speechEvent ROS node definition

This program defines the speechEvent ROS node. The speechEvent ROS node
transcribes Kinyarwanda and English speech utterances in an audio signal
published by the soundDetection ROS node on the /soundDetection/signal ROS
topic, and publishes the transcribed text on the /speechEvent/text ROS topic.

Libraries:
- Ubuntu libraries: cython3 ffmpeg gfortran libopenblas-dev
    libopenblas64-dev patchelf pkg-config python3-testresources
    python3-typing-extensions sox
- Python libraries: nemo, numpy, rospy, scipy, std_msgs, torch

Parameters:
- None

Command-line Parameters:
- None

Configuration File Parameters:
- language                              Kinyarwanda | English
- verboseMode                           true | false
- cuda                                  true | false
- confidence                            0.2
- speechPausePeriod                     1.5
- maxUtteranceLength                    5
- sampleRate                            48000
- heartbeatMsgPeriod                    10

Subscribed Topics and Message Types:
- /soundDetection/signal                std_msgs/Float32MultiArray

Published Topics and Message Types:
- /speechEvent/text                     std_msgs/String

Services Invoked:
- None

Services Advertised and Request Message:
- /speechEvent/set_language             kinyarwanda | english
- /speechEvent/set_enabled              true | false

Input Data Files:
- speech_event_input.dat
- pepper_topics.dat

Output Data Files:
- None

Configuration Files:
- speech_event_configuration.ini

Example Instantiation of the Module:
- rosrun cssr_system speech_event_application.py

Author:     Clifford Onyonka, Carnegie Mellon University Africa
Email:      cliffor2@andrew.cmu.edu
Date:       2025-02-23
Version:    v1.0
"""

import os
import sys

import rospy
import torch

import speech_event_implementation as se_imp
import speech_event_utils as se_utils


SUPPORTED_MODELS = ["conformer-transducer", "parakeet", "whisper"]
RW_MODELS = {
    "conformer-transducer": "stt_rw_conformer_transducer_large.nemo",
    "parakeet": "stt_rw_parakeet-tdt_ctc-110m.nemo",
    "whisper": "whisper_small_rw.pt"
}
EN_MODELS = {
    "conformer-transducer": "stt_en_conformer_transducer_large.nemo",
    "parakeet": "stt_en_parakeet-tdt_ctc-110m.nemo",
    "whisper": "whisper_small_en.pt"
}


if __name__ == "__main__":
    torch.multiprocessing.set_start_method("spawn", force=True)

    current_file_dir = os.path.dirname(__file__)

    topics_file = os.path.join(os.path.dirname(current_file_dir), "data", "pepper_topics.dat")
    config_file = os.path.join(os.path.dirname(current_file_dir), "config", "speech_event_configuration.ini")
    topics = se_utils.parse_config_file(topics_file)
    config = se_utils.parse_config_file(config_file)
    model_name = config["model"].strip().lower()

    if model_name not in SUPPORTED_MODELS:
        print(f"speechEvent: '{model_name}' not supported, supported models are {SUPPORTED_MODELS}")
        sys.exit(1)

    rw_model_path = os.path.join(os.path.dirname(current_file_dir), "models", RW_MODELS[model_name])
    en_model_path = os.path.join(os.path.dirname(current_file_dir), "models", EN_MODELS[model_name])

    se_imp.initialise(config, topics, rw_model_path, en_model_path)

    try:
        se_imp.run()
    except rospy.ROSInterruptException:
        pass
