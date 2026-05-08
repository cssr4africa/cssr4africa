"""
speech_event_implementation.py - audio manipulation functions that support speechEvent

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

import os
import signal
import threading
import time
import sys

import actionlib
import rospy
import torch
import torchaudio
from std_msgs.msg import String, Float32MultiArray

from cssr_system.msg import recognise_speechAction, recognise_speechResult, recognise_speechFeedback
from cssr_system.srv import set_language, set_languageResponse

import speech_event_gui as se_gui
import speech_event_utils as se_utils


# Static config options (not set via config file)
ASR_SAMPLE_RATE = 16000  # of audio required by ASR
NODE_NAME = "speech_event"
PUB_TOPIC = "/speechEvent/recognise_speech_action/result"
RECOGNISE_SPEECH_ACTION = "/speechEvent/recognise_speech_action"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SOUND_DETECTION_DOWN_TEXT = "Error: soundDetection is down"
SOUND_DETECTION_HEALTH_CHECK_PERIOD = 5  # seconds
SPEECH_NOT_RECOGNISED_TEXT = "Error: speech not recognized"
SUPPORTED_LANGUAGES = ["kinyarwanda", "english"]
SUPPORTED_MODELS = ["conformer-transducer", "parakeet", "whisper"]

# Config options set via config file
LANGUAGE = "Kinyarwanda"  # Kinyarwanda or English
MODEL_NAME = "parakeet"  # one of SUPPORTED_MODELS
VERBOSE_MODE = True
CUDA = False
CONFIDENCE = 0.5  # range of [0, 1]
VAD_THRESHOLD = 0.5  # range of [0, 1]
SAMPLE_RATE = 48000  # of audio signal incoming from soundDetection
INTER_UTTERANCE_LEN = 1.0  # seconds
MIN_UTTERANCE_LEN = 1.0  # seconds
MAX_UTTERANCE_LEN = 5.0  # seconds
HEARTBEAT_MSG_PERIOD = 10  # seconds

# Config options set via topics data file
SOUND_DETECTION_TOPIC = "/soundDetection/signal"

# Config options that are resolved dynamically
RW_MODEL_PATH = "/rw.model"
EN_MODEL_PATH = "/en.model"
VAD_MODEL_PATH = "/vad.model"

# Global variables
_model = None
_recognise_speech_action_server = None
_streamed_samples = torch.tensor([], dtype=torch.float32)
_streamed_samples_len = 0
_size_of_streamed_chunks = 0
_min_utterance_len = int(MIN_UTTERANCE_LEN * SAMPLE_RATE)
_max_utterance_len = int(MAX_UTTERANCE_LEN * SAMPLE_RATE)
_aggregate_signal = False
_max_waiting_time = 5.0  # seconds


def _is_sound_detection_running():
    """
    Check if soundDetection signal topic is published

    Returns:
        bool:   True if it's published, False otherwise
    """
    if len([i for i, _ in rospy.get_published_topics() if i.strip() == SOUND_DETECTION_TOPIC]) == 0:
        return False
    return True


def _wait_for_signal_aggregation(feedback):
    """
    Aggregate audio samples being published by sound detection into a signal to
    be passed to the speech recognition model for transcription

    Parameters:
        feedback (object):   used to publish progress to the ROS action client
    """
    prev = _streamed_samples_len
    i = 0

    feedback.progress = 0
    _recognise_speech_action_server.publish_feedback(feedback)

    while True:
        i += 1
        elapsed_time = i * INTER_UTTERANCE_LEN
        time.sleep(INTER_UTTERANCE_LEN)

        feedback.progress = int(elapsed_time * 100 / MAX_UTTERANCE_LEN)
        _recognise_speech_action_server.publish_feedback(feedback)

        if _streamed_samples_len == 0:
            if elapsed_time < _max_waiting_time:
                continue
            break

        if _streamed_samples_len == prev or elapsed_time >= MAX_UTTERANCE_LEN:
            break

        prev = _streamed_samples_len

    feedback.progress = 100
    _recognise_speech_action_server.publish_feedback(feedback)


def _set_transcription_language(language):
    """ Sets the language of operation of speechEvent

    Parameters:
        language:       language speechEvent is to be set to

    Returns:
        int:            1 for success and 0 for failure
    """
    global LANGUAGE, _model

    if language.strip().lower() not in SUPPORTED_LANGUAGES:
        se_utils.log(
            "warning",
            "speechEvent: the language '%s' is not supported, supported "
            "languages are Kinyarwanda and English" % language.strip()
        )
        return 0

    LANGUAGE = language.strip().lower()

    if CUDA:
        if not torch.cuda.is_available():
            se_utils.log("speechEvent: CUDA not available, defaulting to CPU")
        device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    else:
        device = torch.device("cpu")

    _model = se_utils.get_model(MODEL_NAME, RW_MODEL_PATH, EN_MODEL_PATH, LANGUAGE, device)

    se_utils.log(
        "info", f"speechEvent: transcription language set to {LANGUAGE.capitalize()}"
    ) if VERBOSE_MODE else "pass"

    return 1


def _sound_detection_callback(data):
    """ Function that gets called every time a new audio signal is obtained

    Parameters:
        data:           object containing the audio signal
    """
    global _streamed_samples, _streamed_samples_len, _size_of_streamed_chunks

    if not _aggregate_signal:
        return

    if _size_of_streamed_chunks == 0:
        _size_of_streamed_chunks = len(data.data)

    _streamed_samples = torch.cat((_streamed_samples, torch.tensor(data.data, dtype=torch.float32)))
    _streamed_samples_len += _size_of_streamed_chunks


def _set_language_srv_handler(req):
    """ Function that gets called every time the set language ROS service is
    invoked. It sets the language of operation of speechEvent.

    Parameters:
        req:            request object containing the language speechEvent is to be set to

    Returns:
        Response:   Response(1) for sucess, Response(0) for failure
    """
    return set_languageResponse(
        _set_transcription_language(req.language)
    )


def _recognise_speech_action_handler(goal):
    """ Function that gets called every time the peech action server is run. It
    captures an audio signal and generates a text transcription from any speech
    in the audio.

    Parameters:
        goal (object):  message object containing arguments sent by the client
    """
    global _streamed_samples, _streamed_samples_len, _size_of_streamed_chunks, \
        _aggregate_signal, _max_waiting_time

    _max_waiting_time = goal.wait
    feedback = recognise_speechFeedback()
    result = recognise_speechResult()
    start_time = time.time()

    if not _is_sound_detection_running():
        se_utils.log("warning", f"speechEvent: can't connect to {SOUND_DETECTION_TOPIC}")

        feedback.progress = 100
        _recognise_speech_action_server.publish_feedback(feedback)

        result.transcription = SOUND_DETECTION_DOWN_TEXT
        _recognise_speech_action_server.set_aborted(result)

        return

    _streamed_samples = torch.tensor([], dtype=torch.float32)
    _streamed_samples_len = 0
    _size_of_streamed_chunks = 0
    _aggregate_signal = True

    t = threading.Thread(target=_wait_for_signal_aggregation, args=(feedback,))
    t.start()
    t.join()
    _aggregate_signal = False

    if _streamed_samples.shape[0] == 0:
        result.transcription = ""
        _recognise_speech_action_server.set_succeeded(result)
        return

    resampler = torchaudio.transforms.Resample(orig_freq=SAMPLE_RATE, new_freq=ASR_SAMPLE_RATE)
    device = torch.device("cpu") if not CUDA else \
        torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    resampled = resampler(_streamed_samples)
    resampled = resampled.to(device)
    normalised = resampled / resampled.abs().max()
    hypotheses = se_utils.get_hypotheses(MODEL_NAME, _model, normalised)
    transcription, log_message = se_utils.get_transcription(
        MODEL_NAME, hypotheses, SPEECH_NOT_RECOGNISED_TEXT, CONFIDENCE
    )

    if VERBOSE_MODE:
        se_utils.log("info", f"{log_message} - {round(time.time() - start_time, 4)} seconds")

    result.transcription = transcription if transcription != SPEECH_NOT_RECOGNISED_TEXT else ""
    _recognise_speech_action_server.set_succeeded(result)


def run():
    """
    Run a speechEvent ROS node
    """
    global _recognise_speech_action_server, _min_utterance_len, _max_utterance_len

    rospy.Service(SET_LANGUAGE_SERVICE, set_language,_set_language_srv_handler)
    rospy.Subscriber(SOUND_DETECTION_TOPIC, Float32MultiArray, _sound_detection_callback)
    rospy.Timer(
        rospy.Duration(HEARTBEAT_MSG_PERIOD),
        lambda _: se_utils.log("info", "speechEvent: running")
    )
    _recognise_speech_action_server = actionlib.SimpleActionServer(
        RECOGNISE_SPEECH_ACTION, recognise_speechAction, _recognise_speech_action_handler,
        auto_start=False
    )

    se_utils.log(
        "info",
        f"""speechEvent v1.1

        \r{' ' * 28}This project is funded by the African Engineering and Technology Network (Afretec)
        \r{' ' * 28}Inclusive Digital Transformation Research Grant Programme.

        \r{' ' * 28}Website: www.cssr4africa.org

        \r{' ' * 28}This program comes with ABSOLUTELY NO WARRANTY.
        """
    )
    se_utils.log("info", "speechEvent: start-up")

    if _is_sound_detection_running():
        se_utils.log("info", f"speechEvent: subscribed to {SOUND_DETECTION_TOPIC}")
    else:
        se_utils.log(
            "warning", f"speechEvent: can't connect to {SOUND_DETECTION_TOPIC}"
        )

    se_utils.log("info", f"speechEvent: {SET_LANGUAGE_SERVICE} service advertised")

    _set_transcription_language(LANGUAGE)
    _min_utterance_len = int(MIN_UTTERANCE_LEN * SAMPLE_RATE)
    _max_utterance_len = int(MAX_UTTERANCE_LEN * SAMPLE_RATE)

    _recognise_speech_action_server.start()

    se_utils.log("info", f"speechEvent: {RECOGNISE_SPEECH_ACTION} action server is ready")

    if VERBOSE_MODE:
        display_process = torch.multiprocessing.Process(target=se_gui.GUI.run, args=(PUB_TOPIC,))
        display_process.start()

    def stop_processes(_, __):
        if VERBOSE_MODE:
            display_process.terminate()
            display_process.join()
        raise rospy.ROSInterruptException()

    signal.signal(signal.SIGINT, stop_processes)
    signal.signal(signal.SIGTERM, stop_processes)

    rospy.spin()


def initialise(config, topics, rw_model_path, en_model_path, vad_model_path):
    """ Make preparatory initialisations before running a speechEvent ROS node

    Patameters:
        config:             object containing configuration arguments
        topics:             object containing ROS topics to be subscribed to
        rw_model_path:      path to Kinyarwanda silero_vadASR model
        en_model_path:      path to English ASR model
        vad_model_path:     path to Silero voice activity detection model
    """
    global LANGUAGE, MODEL_NAME, VERBOSE_MODE, CUDA, CONFIDENCE, VAD_THRESHOLD, SAMPLE_RATE
    global INTER_UTTERANCE_LEN, MIN_UTTERANCE_LEN, MAX_UTTERANCE_LEN, HEARTBEAT_MSG_PERIOD
    global RW_MODEL_PATH, EN_MODEL_PATH, VAD_MODEL_PATH, SOUND_DETECTION_TOPIC

    rospy.init_node(NODE_NAME, anonymous=True)

    if config["verboseMode"].strip().lower() not in ["true", "false"]:
        se_utils.log(
            "error",
            "speechEvent: the verboseMode '%s' is not supported, supported "
            "verboseModes are true and false" % config["verboseMode"].strip()
        )
        sys.exit(1)

    if config["language"].strip().lower() not in SUPPORTED_LANGUAGES:
        se_utils.log(
            "error",
            "speechEvent: the language '%s' is not supported, supported "
            "languages are Kinyarwanda and English" % config["language"].strip()
        )
        sys.exit(1)

    if config["model"].strip().lower() not in SUPPORTED_MODELS:
        se_utils.log(
            "error",
            f"speechEvent: '{config['model'].strip()}' not supported, supported "
            f"models are {SUPPORTED_MODELS}"
        )
        sys.exit(1)

    if config["cuda"].strip().lower() not in ["true", "false"]:
        se_utils.log(
            "error",
            "speechEvent: the value '%s' for cuda is not supported, supported "
            "values are true and false" % config["cuda"].strip()
        )
        sys.exit(1)

    if not os.path.exists(rw_model_path):
        se_utils.log(
            "error",
            "speechEvent: the Kinyarwanda ASR model is absent from the models directory"
        )
        sys.exit(1)

    if not os.path.exists(en_model_path):
        se_utils.log(
            "error",
            "speechEvent: the English ASR model is absent from the models directory"
        )
        sys.exit(1)

    LANGUAGE = config["language"].strip().lower()
    MODEL_NAME = config["model"].strip().lower()
    VERBOSE_MODE = True if config["verboseMode"].strip().lower() == "true" else False
    CUDA = True if config["cuda"].strip().lower() == "true" else False
    CONFIDENCE = float(config["confidence"].strip())
    VAD_THRESHOLD = float(config["vadThreshold"].strip())
    SAMPLE_RATE = int(config["sampleRate"].strip())
    INTER_UTTERANCE_LEN = float(config["interUtteranceLen"].strip())
    MIN_UTTERANCE_LEN = float(config["minUtteranceLen"].strip())
    MAX_UTTERANCE_LEN = float(config["maxUtteranceLen"].strip())
    HEARTBEAT_MSG_PERIOD = int(config["heartbeatMsgPeriod"].strip())
    SOUND_DETECTION_TOPIC = topics["soundDetection"].strip()
    RW_MODEL_PATH = rw_model_path
    EN_MODEL_PATH = en_model_path
    VAD_MODEL_PATH = vad_model_path
