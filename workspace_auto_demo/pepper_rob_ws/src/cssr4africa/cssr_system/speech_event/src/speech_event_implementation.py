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

import multiprocessing
import os
import signal
import sys
import time

import nemo.collections.asr as nemo_asr
import nemo.utils.nemo_logging as nemo_logging
import rospy
import torch
import torchaudio
from std_msgs.msg import String, Float32MultiArray

from cssr_system.srv import set_enabled, set_enabledResponse
from cssr_system.srv import set_language, set_languageResponse

import speech_event_gui as se_gui


# Static config options (not set via config file)
NODE_NAME = "speech_event"
PUB_TOPIC = "/speechEvent/text"
SET_ENABLED_SERVICE = "/speechEvent/set_enabled"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SOUND_DETECTION_TOPIC_CHECK_PERIOD = 0.05  # seconds
SOUND_DETECTION_HEALTH_CHECK_PERIOD = 5  # seconds
SPEECH_NOT_RECOGNISED_TEXT = "Error: speech not recognized"
SOUND_DETECTION_DOWN_TEXT = "Error: soundDetection is down"
NEMO_SAMPLE_RATE = 16000  # sampling rate of audio required by nemo ASR models
IS_TRANSCRIPTION_ENABLED = True
TRANSCRIPTION_WINDOW = 1  # seconds

# Config options set via config file
LANGUAGE = "Kinyarwanda"  # Kinyarwanda or English
VERBOSE_MODE = True
CUDA = False
CONFIDENCE = 0.5  # on a scale of 0 to 1
SAMPLE_RATE = 48000  # sampling rate of audio signal incoming from soundDetection
HEARTBEAT_MSG_PERIOD = 10  # seconds

# Config options set via topics data file
SOUND_DETECTION_TOPIC = "/soundDetection/signal"

# Config options that are resolved dynamically
RW_MODEL_PATH = "/rw.nemo"
EN_MODEL_PATH = "/en.nemo"

# Global variables
_device = None  # cpu or cuda
_model = None  # ASR model
_publisher = None
_streamed_samples = torch.tensor([], dtype=torch.float32)
_last_audio_received_at = None
_first_audio_received_at = None


def _get_audio_transcription(samples):
    """ Extract a text transcription from a wav file

    Parameters:
        samples (torch.tensor): audio signal

    Returns:
        str:    the keyword spotted from the audio
    """
    samples = samples.reshape((1, -1))
    samples = torchaudio.transforms.Resample(orig_freq=SAMPLE_RATE, new_freq=NEMO_SAMPLE_RATE)(samples)
    samples = (samples / samples.abs().max()).to(_device)
    samples_length = torch.tensor([samples.shape[1]], dtype=torch.int64).to(_device)

    with torch.no_grad():
        logits = _model(input_signal=samples, input_signal_length=samples_length)

    probs = torch.softmax(logits, dim=1).squeeze()
    pred_index = torch.argmax(probs).item()
    confidence = round(probs[pred_index].item(), 4)
    pred_label = _model.cfg.labels[pred_index]
    log_message_template = "speechEvent: spotted keyword -> {text} (confidence: {conf})"

    if confidence >= CONFIDENCE:
        log_message = log_message_template.format(text=pred_label, conf=confidence)
    else:
        log_message = log_message_template.format(
            text=f"{pred_label} [ignored]", conf=confidence
        )
        pred_label = SPEECH_NOT_RECOGNISED_TEXT

    rospy.loginfo(log_message) if VERBOSE_MODE else "pass"

    return pred_label


def _set_transcription_language(language):
    """ Sets the language of operation of speechEvent

    Parameters:
        language:   language speechEvent is to be set to

    Returns:
        int:        1 for success and 0 for failure
    """
    global LANGUAGE, _device, _model

    if language.strip().lower() not in ["kinyarwanda", "english"]:
        rospy.logwarn(
            "speechEvent: the language '%s' is not supported, supported "
            "languages are Kinyarwanda and English" % language.strip()
        )
        return 0

    if CUDA:
        if not torch.cuda.is_available():
            rospy.logwarn(
                "speechEvent: CUDA not available, defaulting to CPU"
            )
        _device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    else:
        _device = torch.device("cpu")

    LANGUAGE = language.strip().lower()
    _model = nemo_asr.models.EncDecRNNTBPEModel.restore_from(
        restore_path=RW_MODEL_PATH if LANGUAGE == "kinyarwanda" else EN_MODEL_PATH,
        map_location=_device
    )
    _model.eval()

    rospy.loginfo(
        f"speechEvent: language set to {LANGUAGE.capitalize()}"
    ) if VERBOSE_MODE else "pass"

    return 1


def _trigger_audio_transcription(event):
    """
    Trigger the audio tanscription procedure when there is a prolonged silence on
    the /soundDetection/signal ROS topic (based on a time threshold), which
    transcribes speech in the audio samples retrieved from the /soundDection/signal
    ROS topic and publishes the transcribed text on the /speechEvent/text ROS topic

    Parameters:
        event:  timer event (not used)
    """
    global _streamed_samples, _last_audio_received_at

    if not IS_TRANSCRIPTION_ENABLED:
        _streamed_samples = torch.tensor([], dtype=torch.float32)
        _last_audio_received_at = None
        return

    if _last_audio_received_at is None:
        return

    samples_to_transcribe = _streamed_samples[:int(TRANSCRIPTION_WINDOW * SAMPLE_RATE)]

    if samples_to_transcribe.shape[0] < SAMPLE_RATE:
        return

    transcription = _get_audio_transcription(samples_to_transcribe)
    _publisher.publish(transcription) if transcription != SPEECH_NOT_RECOGNISED_TEXT else "pass"

    rospy.loginfo(
        "speechEvent: transcription process (plus utterance length) has taken "
        f"{round(time.time() - _first_audio_received_at, 4)} seconds"
    ) if VERBOSE_MODE else "pass"

    _streamed_samples = _streamed_samples[(TRANSCRIPTION_WINDOW * SAMPLE_RATE) // 2:]
    _last_audio_received_at = None


def _sound_detection_callback(data):
    """ Function that gets called every time a new audio signal is obtained,
    transcribing speech utterances within the received audio signal

    Parameters:
        data:   object containing the audio signal
    """
    global _streamed_samples, _last_audio_received_at, _first_audio_received_at

    time_now = time.time()

    if _last_audio_received_at is None:
        _first_audio_received_at = time_now

    _last_audio_received_at = time_now
    _streamed_samples = torch.cat((_streamed_samples, torch.tensor(data.data, dtype=torch.float32)), dim=0)


def _set_language_srv_handler(req):
    """ Function that gets called every time the /speechEvent/set_language ROS
    service is invoked. It sets the language of operation of speechEvent.

    Parameters:
        req:   request object containing the language speechEvent is to be set to
    """
    return set_languageResponse(_set_transcription_language(req.language))


def _set_enabled_srv_handler(req):
    """ Function that gets called every time the /speechEvent/set_enabled ROS
    service is invoked. It sets the status of the transcription process to either
    enabled or disabled.

    Parameters:
        req:   request object containing the status the transcription process is
            to be set to
    """
    global IS_TRANSCRIPTION_ENABLED

    status = req.status.strip().lower()

    if status not in ["true", "false"]:
        rospy.logwarn(
            "speechEvent: the status '%s' is not supported, supported status "
            "settings are true and false" % req.status.strip()
        )
        return set_enabledResponse(0)

    IS_TRANSCRIPTION_ENABLED = True if status == "true" else False

    rospy.loginfo(
        f"speechEvent: transcription set to {'enabled' if IS_TRANSCRIPTION_ENABLED else 'disabled'}"
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
    rospy.Service(SET_ENABLED_SERVICE, set_enabled, _set_enabled_srv_handler)
    rospy.Service(SET_LANGUAGE_SERVICE, set_language, _set_language_srv_handler)
    rospy.Subscriber(SOUND_DETECTION_TOPIC, Float32MultiArray, _sound_detection_callback)
    rospy.Timer(
        rospy.Duration(SOUND_DETECTION_TOPIC_CHECK_PERIOD), _trigger_audio_transcription
    )
    rospy.Timer(
        rospy.Duration(HEARTBEAT_MSG_PERIOD),
        lambda _: rospy.loginfo("speechEvent: running")
    )
    rospy.Timer(
        rospy.Duration(SOUND_DETECTION_HEALTH_CHECK_PERIOD),
        _publish_sound_detection_is_down
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

    if VERBOSE_MODE:
        display_process = multiprocessing.Process(target=se_gui.GUI.run, args=(PUB_TOPIC,))

        def kill(_, __):
            display_process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, kill)
        display_process.start()

    rospy.spin()


def initialise(config, topics, rw_model_path, en_model_path):
    """ Make preparatory initialisations before running a speechEvent ROS node

    Patameters:
        config:             object containing configuration arguments
        topics:             object containing ROS topics to be subscribed to
        rw_model_path:      path to Kinyarwanda ASR model
        en_model_path:      path to English ASR model
    """
    global LANGUAGE, VERBOSE_MODE, CUDA, CONFIDENCE, SAMPLE_RATE, HEARTBEAT_MSG_PERIOD
    global SOUND_DETECTION_TOPIC
    global RW_MODEL_PATH, EN_MODEL_PATH
    global _publisher

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
    VERBOSE_MODE = True if config["verboseMode"].strip().lower() == "true" else False
    CUDA = True if config["cuda"].strip().lower() == "true" else False
    CONFIDENCE = float(config["confidence"].strip())
    SAMPLE_RATE = int(config["sampleRate"].strip())
    HEARTBEAT_MSG_PERIOD = int(config["heartbeatMsgPeriod"].strip())
    SOUND_DETECTION_TOPIC = topics["soundDetection"].strip()
    RW_MODEL_PATH = rw_model_path
    EN_MODEL_PATH = en_model_path

    nemo_logging.Logger().set_verbosity(nemo_logging.Logger.ERROR)
    _set_transcription_language(LANGUAGE)
    _publisher = rospy.Publisher(PUB_TOPIC, String, queue_size=10)
