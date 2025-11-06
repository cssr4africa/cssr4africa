"""
speech_event_implementation.py - audio manipulation functions that support speechEvent

Author:     Clifford Onyonka
Date:       2025-02-23
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import ctypes
import os
import signal
import sys
from functools import partial

import rospy
import torch
from std_msgs.msg import String, Float32MultiArray

from cssr_system.srv import set_enabled, set_enabledResponse
from cssr_system.srv import set_language, set_languageResponse

import speech_event_gui as se_gui
from speech_event_transcription import LOG_LEVELS, run_transcriptions


# Static config options (not set via config file)
NODE_NAME = "speech_event"
PUB_TOPIC = "/speechEvent/text"
SET_ENABLED_SERVICE = "/speechEvent/set_enabled"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SOUND_DETECTION_HEALTH_CHECK_PERIOD = 5  # seconds
SOUND_DETECTION_DOWN_TEXT = "Error: soundDetection is down"
LOG_LEVELS_INV = {v: k for k, v in LOG_LEVELS.items()}
LOG_FUNCTIONS = {
    "debug": rospy.logdebug,
    "info": rospy.loginfo,
    "warning": rospy.logwarn,
    "error": rospy.logerr,
    "critical": rospy.logfatal
}
SUPPORTED_MODELS = ["conformer-transducer", "parakeet", "whisper"]

# Config options set via config file
LANGUAGE = "Kinyarwanda"  # Kinyarwanda or English
MODEL_NAME = "parakeet"  # one of SUPPORTED_MODELS
VERBOSE_MODE = True
CUDA = False
IS_ENABLED = True
CONFIDENCE = 0.5  # on a scale of 0 t0 1
INTER_UTTERANCE_LEN = 0.5  # seconds
VAD_THRESHOLD = 0.5  # range of [0, 1]
SAMPLE_RATE = 48000  # of audio signal incoming from soundDetection
MIN_UTTERANCE_LEN = 1  # seconds
HEARTBEAT_MSG_PERIOD = 10  # seconds

# Config options set via topics data file
SOUND_DETECTION_TOPIC = "/soundDetection/signal"

# Config options that are resolved dynamically
RW_MODEL_PATH = "/rw.model"
EN_MODEL_PATH = "/en.model"
VAD_MODEL_PATH = "/vad.model"
AUDIO_MAX_LEN = SAMPLE_RATE * 60  # number of samples

# Global variables
_publisher = None
_mp_streamed_samples = torch.zeros(AUDIO_MAX_LEN, dtype=torch.float32).share_memory_()
_mp_streamed_samples_list = []
_mp_streamed_samples_len = 0
_size_of_streamed_chunks = 0
_min_utterance_len = int(MIN_UTTERANCE_LEN * SAMPLE_RATE)


def _log(_, mp_log_lock, mp_log_level, mp_log_message):
    """ Log messages from different processes using ROS' logging system

    Parameters:
        _:              ignored, and therefore not used
        mp_log_lock:    [multi-processing] log lock object
        mp_log_level:   [multi-processing] log level variable
        mp_log_message: [multi-processing] log message variable
    """
    with mp_log_lock:
        level = LOG_LEVELS_INV[mp_log_level.value]
        message = mp_log_message.value.decode("UTF-8")

    if level != "none":
        LOG_FUNCTIONS[level](message)

    with mp_log_lock:
        mp_log_level.value = LOG_LEVELS["none"]


def _publish(_, mp_pub_pipe):
    """ Publish transcriptions to the /speechEvent/text ROS topic

    Parameters:
        _:                      ignored, and therefore not used
        mp_pub_pipe:            [multi-processing] pub pipe object
    """
    if not mp_pub_pipe.poll():
        return

    try:
        transcription = mp_pub_pipe.recv()
    except EOFError:
        return

    if len(transcription.strip()) > 0 and IS_ENABLED:
        _publisher.publish(transcription)


def _set_transcription_language(language, mp_misc_lock, mp_lang):
    """ Sets the language of operation of speechEvent

    Parameters:
        language:       language speechEvent is to be set to
        mp_misc_lock:   [multi-processing] miscellaneous lock object
        mp_lang:        [multi-processing] language variable

    Returns:
        int:            1 for success and 0 for failure
    """
    global LANGUAGE

    if language.strip().lower() not in ["kinyarwanda", "english"]:
        rospy.logwarn(
            "speechEvent: the language '%s' is not supported, supported "
            "languages are Kinyarwanda and English" % language.strip()
        )
        return 0

    LANGUAGE = language.strip().lower()

    with mp_misc_lock:
        mp_lang.value = LANGUAGE.encode("UTF-8")

    rospy.loginfo(
        f"speechEvent: language set to {LANGUAGE.capitalize()}"
    ) if VERBOSE_MODE else "pass"

    return 1


def _sound_detection_callback(data, mp_tensor_lock, mp_current_idx):
    """ Function that gets called every time a new audio signal is obtained,
    transcribing speech utterances within the received audio signal

    Parameters:
        data:           object containing the audio signal
        mp_tensor_lock: [multi-processing] lock object
        mp_current_idx: [multi-processing] current cursor location
    """
    global _mp_streamed_samples, _mp_streamed_samples_list, _mp_streamed_samples_len
    global _size_of_streamed_chunks

    if not IS_ENABLED:
        return

    if _size_of_streamed_chunks == 0:
        _size_of_streamed_chunks = len(data.data)

    _mp_streamed_samples_list.extend(data.data)
    _mp_streamed_samples_len += _size_of_streamed_chunks

    if _mp_streamed_samples_len < _min_utterance_len * 0.5:
        return

    audio_tensor = torch.tensor(_mp_streamed_samples_list, dtype=torch.float32)
    audio_len = audio_tensor.shape[0]
    _mp_streamed_samples[mp_current_idx.value: mp_current_idx.value+audio_len] = audio_tensor

    with mp_tensor_lock:
        mp_current_idx.value += audio_len

    if mp_current_idx.value > AUDIO_MAX_LEN - audio_len:
        mp_current_idx.value = 0

    _mp_streamed_samples_list = []
    _mp_streamed_samples_len = 0


def _set_language_srv_handler(req, mp_misc_lock, mp_lang):
    """ Function that gets called every time the /speechEvent/set_language ROS
    service is invoked. It sets the language of operation of speechEvent.

    Parameters:
        req:            request object containing the language speechEvent is to be set to
        mp_misc_lock:   [multi-processing] miscellaneous lock object
        mp_lang:        [multi-processing] language variable
    """
    return set_languageResponse(
        _set_transcription_language(req.language, mp_misc_lock, mp_lang)
    )


def _set_enabled_srv_handler(req):
    """ Function that gets called every time the /speechEvent/set_enabled ROS
    service is invoked. It sets the status of the transcription process to either
    enabled or disabled.

    Parameters:
        req:            request object containing the status the transcription process is to be set to
    """
    global IS_ENABLED

    status = req.status.strip().lower()

    if status not in ["true", "false"]:
        rospy.logwarn(
            "speechEvent: the status '%s' is not supported, supported status "
            "settings are true and false" % req.status.strip()
        )
        return set_enabledResponse(0)

    IS_ENABLED = True if status == "true" else False

    rospy.loginfo(
        f"speechEvent: transcription set to {'enabled' if IS_ENABLED else 'disabled'}"
    ) if VERBOSE_MODE else "pass"

    return set_enabledResponse(1)


def _is_sound_detection_running(sound_detection_topic):
    """ Check if the /soundDection/signal ROS topic is published

    Parameters:
        sound_detection_topic (str):    the ROS topic

    Returns:
        bool:   True if it's published, False otherwise
    """
    is_running = False
    for (topic, _) in rospy.get_published_topics():
        if topic.strip() == sound_detection_topic.strip():
            is_running = True
            break
    return is_running


def _publish_sound_detection_is_down(_):
    """
    Publish on /speechEvent/text that /soundDetection/signal is not published

    Parameter:
        _:  ignored, and therefore not used
    """
    if len([i for i, __ in rospy.get_published_topics() if i == SOUND_DETECTION_TOPIC]) == 0:
        _publisher.publish(SOUND_DETECTION_DOWN_TEXT)
        rospy.logwarn(f"speechEvent: can't connect to {SOUND_DETECTION_TOPIC}")


def run():
    """
    Run a speechEvent ROS node
    """
    global _publisher, _min_utterance_len

    mp_tensor_lock = torch.multiprocessing.Lock()
    mp_misc_lock = torch.multiprocessing.Lock()
    mp_log_lock = torch.multiprocessing.Lock()
    mp_pub_pipe_parent, mp_pub_pipe_child = torch.multiprocessing.Pipe()
    mp_current_idx = torch.multiprocessing.Value("i", 0)
    mp_lang = torch.multiprocessing.Array(ctypes.c_char, 32)
    mp_log_level = torch.multiprocessing.Value("i", 0)
    mp_log_message = torch.multiprocessing.Array(ctypes.c_char, AUDIO_MAX_LEN)

    mp_lang.value = LANGUAGE.encode("UTF-8")

    rospy.Service(SET_ENABLED_SERVICE, set_enabled, _set_enabled_srv_handler)
    rospy.Service(
        SET_LANGUAGE_SERVICE,
        set_language,
        partial(_set_language_srv_handler, mp_misc_lock=mp_misc_lock, mp_lang=mp_lang)
    )
    rospy.Subscriber(
        SOUND_DETECTION_TOPIC,
        Float32MultiArray,
        partial(_sound_detection_callback, mp_tensor_lock=mp_tensor_lock, mp_current_idx=mp_current_idx)
    )
    rospy.Timer(
        rospy.Duration(HEARTBEAT_MSG_PERIOD),
        lambda _: rospy.loginfo("speechEvent: running")
    )
    rospy.Timer(
        rospy.Duration(SOUND_DETECTION_HEALTH_CHECK_PERIOD),
        _publish_sound_detection_is_down
    )
    rospy.Timer(
        rospy.Duration(nsecs=int(1e3)),
        partial(_log, mp_log_lock=mp_log_lock, mp_log_level=mp_log_level, mp_log_message=mp_log_message)
    )
    rospy.Timer(
        rospy.Duration(nsecs=int(1e3)),
        partial(_publish, mp_pub_pipe=mp_pub_pipe_parent)
    )

    rospy.loginfo(
        f"""speechEvent v1.0

        \r{' ' * 28}This project is funded by the African Engineering and Technology Network (Afretec)
        \r{' ' * 28}Inclusive Digital Transformation Research Grant Programme.

        \r{' ' * 28}Website: www.cssr4africa.org

        \r{' ' * 28}This program comes with ABSOLUTELY NO WARRANTY.
        """
    )
    rospy.loginfo("speechEvent: start-up")

    if _is_sound_detection_running(SOUND_DETECTION_TOPIC):
        rospy.loginfo(f"speechEvent: subscribed to {SOUND_DETECTION_TOPIC}")
    else:
        rospy.logwarn(
            f"speechEvent: can't connect to {SOUND_DETECTION_TOPIC}, "
            f"will keep checking every {SOUND_DETECTION_HEALTH_CHECK_PERIOD} seconds"
        )

    rospy.loginfo(f"speechEvent: transcription language set to {LANGUAGE}")
    rospy.loginfo(f"speechEvent: {SET_ENABLED_SERVICE} service advertised")
    rospy.loginfo(f"speechEvent: {SET_LANGUAGE_SERVICE} service advertised")

    _set_transcription_language(LANGUAGE, mp_misc_lock, mp_lang)
    _publisher = rospy.Publisher(PUB_TOPIC, String, queue_size=10)
    _min_utterance_len = int(MIN_UTTERANCE_LEN * SAMPLE_RATE)

    transcription_process = torch.multiprocessing.Process(
        target=run_transcriptions,
        args=(
            _mp_streamed_samples, mp_tensor_lock, mp_misc_lock, mp_current_idx,
            mp_lang, mp_log_lock, mp_log_level, mp_log_message, mp_pub_pipe_child,
            CUDA, RW_MODEL_PATH, EN_MODEL_PATH, SAMPLE_RATE, _min_utterance_len,
            AUDIO_MAX_LEN, CONFIDENCE, VERBOSE_MODE, MODEL_NAME, INTER_UTTERANCE_LEN,
            VAD_MODEL_PATH, VAD_THRESHOLD
        )
    )
    transcription_process.start()

    if VERBOSE_MODE:
        display_process = torch.multiprocessing.Process(target=se_gui.GUI.run, args=(PUB_TOPIC,))
        display_process.start()

    def stop_processes(_, __):
        transcription_process.terminate()
        transcription_process.join()
        if VERBOSE_MODE:
            display_process.terminate()
            display_process.join()
        sys.exit(0)

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
    global LANGUAGE, MODEL_NAME, VERBOSE_MODE, CUDA, IS_ENABLED, CONFIDENCE
    global INTER_UTTERANCE_LEN, VAD_THRESHOLD, SAMPLE_RATE, MIN_UTTERANCE_LEN
    global HEARTBEAT_MSG_PERIOD, RW_MODEL_PATH, EN_MODEL_PATH, VAD_MODEL_PATH
    global SOUND_DETECTION_TOPIC

    rospy.init_node(NODE_NAME, anonymous=True)

    if config["verboseMode"].strip().lower() not in ["true", "false"]:
        rospy.logerr(
            "speechEvent: the verboseMode '%s' is not supported, supported "
            "verboseModes are true and false" % config["verboseMode"].strip()
        )
        sys.exit(1)

    if config["language"].strip().lower() not in ["kinyarwanda", "english"]:
        rospy.logerr(
            "speechEvent: the language '%s' is not supported, supported "
            "languages are Kinyarwanda and English" % config["language"].strip()
        )
        sys.exit(1)

    if config["model"].strip().lower() not in SUPPORTED_MODELS:
        rospy.logerr(
            f"speechEvent: '{config['model'].strip()}' not supported, supported "
            f"models are {SUPPORTED_MODELS}"
        )
        sys.exit(1)

    if config["cuda"].strip().lower() not in ["true", "false"]:
        rospy.logerr(
            "speechEvent: the value '%s' for cuda is not supported, supported "
            "values are true and false" % config["cuda"].strip()
        )
        sys.exit(1)

    if not os.path.exists(rw_model_path):
        rospy.logerr(
            "speechEvent: the Kinyarwanda ASR model is absent from the models directory"
        )
        sys.exit(1)

    if not os.path.exists(en_model_path):
        rospy.logerr(
            "speechEvent: the English ASR model is absent from the models directory"
        )
        sys.exit(1)

    LANGUAGE = config["language"].strip().lower()
    MODEL_NAME = config["model"].strip().lower()
    VERBOSE_MODE = True if config["verboseMode"].strip().lower() == "true" else False
    CUDA = True if config["cuda"].strip().lower() == "true" else False
    IS_ENABLED = True if config["is_enabled"].strip().lower() == "true" else False
    CONFIDENCE = float(config["confidence"].strip())
    INTER_UTTERANCE_LEN = float(config["interUtteranceLen"].strip())
    VAD_THRESHOLD = float(config["vadThreshold"].strip())
    SAMPLE_RATE = int(config["sampleRate"].strip())
    MIN_UTTERANCE_LEN = float(config["minUtteranceLen"].strip())
    HEARTBEAT_MSG_PERIOD = int(config["heartbeatMsgPeriod"].strip())
    SOUND_DETECTION_TOPIC = topics["soundDetection"].strip()
    RW_MODEL_PATH = rw_model_path
    EN_MODEL_PATH = en_model_path
    VAD_MODEL_PATH = vad_model_path
