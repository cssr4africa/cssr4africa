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

import io
import multiprocessing
import os
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
import uuid

import nemo.collections.asr as nemo_asr
import nemo.utils.nemo_logging as nemo_logging
import numpy as np
import rospy
import torch
from scipy.io import wavfile
from scipy.signal import resample
from std_msgs.msg import String, Float32MultiArray

from cssr_system.srv import set_enabled, set_enabledResponse
from cssr_system.srv import set_language, set_languageResponse


# Static config options (not set via config file)
NODE_NAME = "speech_event"
PUB_TOPIC = "/speechEvent/text"
SET_ENABLED_SERVICE = "/speechEvent/set_enabled"
SET_LANGUAGE_SERVICE = "/speechEvent/set_language"
SOUND_DETECTION_TOPIC_CHECK_PERIOD = 0.05  # seconds
SOUND_DETECTION_HEALTH_CHECK_PERIOD = 5  # seconds
ROS_LOGGER_THROTTLE_SPEECH_NOT_DETECTED = 1  # seconds
ROS_LOGGER_THROTTLE_SPEECH_DETECTED = 1  # seconds
SPEECH_NOT_RECOGNISED_TEXT = "Error: speech not recognized"
SOUND_DETECTION_DOWN_TEXT = "Error: soundDetection is down"
NEMO_SAMPLE_RATE = 16000  # of audio required by nemo ASR
IS_TRANSCRIPTION_ENABLED = True

# Config options set via config file
LANGUAGE = "Kinyarwanda"  # Kinyarwanda or English
VERBOSE_MODE = True
CUDA = False
CONFIDENCE = 0.5  # on a scale of 0 t0 1
SPEECH_PAUSE_PERIOD = 1.5  # seconds
MAX_UTTERANCE_LENGTH = 5  # seconds
SAMPLE_RATE = 48000  # of audio signal incoming from soundDetection
HEARTBEAT_MSG_PERIOD = 10  # seconds
AUDIO_STORAGE_DIR = "data/audio_storage/"

# Config options set via topics data file
SOUND_DETECTION_TOPIC = "/soundDetection/signal"

# Config options that are resolved dynamically
RW_MODEL_PATH = "/stt_rw_conformer_transducer_large.nemo"
EN_MODEL_PATH = "/stt_en_conformer_transducer_large.nemo"

# Global variables
_model = None
_publisher = None
_streamed_samples = np.array([], dtype=np.float32)
_last_audio_received_at = None
_first_audio_received_at = None


class _GUI:
    def _get_main_window(title):
        """ Return the main top-level Tkinter widget

        Parameters:
            title (str): the title of the top-level window

        Returns:
            tk.Tk:  top-level Tkinter window
        """
        window = tk.Tk()
        window.title(title)
        window.minsize(720, 480)

        return window


    def _get_main_frame(window):
        """ Return the main frame widget that all other widgets are to reside in

        Parameters:
            window (Tk): the top-level window that wraps the main frame

        Returns:
            tk.Frame:   main frame widget
        """
        frame = tk.Frame(window)
        frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        frame.pack(side="top", fill="both", expand=True)
        
        return frame


    def _listen_on_topic(window, text, process):
        """ A forever running loop that listens on the /speechEvent/text ROS topic
        and updates the GUI with any text that gets published on the topic

        Parameters:
            window (tk.Tk):             the main/root Tkinter window
            text (tk.Label):            the Tkinter widget to be updated with text
                transcriptions published on /speechEvent/text
            process (subprocess.Popen): process that reads text published on
                /speechEvent/text

        Returns:
            None
        """
        while True:
            if process.poll():
                break
            stdout = process.stdout.readline().decode("UTF-8")
            window.after(
                0,
                lambda: text.configure(text=stdout[7:-2]) if "data" in stdout else "pass"
            )
            time.sleep(1)

    def run():
        """
        Run GUI application to display text transcriptions being published on the
        /speechEvent/text ROS topic
        """
        window = _GUI._get_main_window(f"SpeechEvent output (rostopic echo {PUB_TOPIC})")
        frame = _GUI._get_main_frame(window)
        process = subprocess.Popen(
            ["rostopic", "echo", PUB_TOPIC],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        text = tk.Label(frame, font=("Helvetica", 16))
        text.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        text.pack(side="top", fill="both", expand=True)

        topic_p = threading.Thread(target=_GUI._listen_on_topic, args=(window, text, process))
        topic_p.start()
        window.mainloop()
        topic_p.join()

        window.destroy()
        process.kill()


def _save_audio(sample_rate, samples):
    """ Save an array of audio samples as a wav file

    Parameters:
        sample_rate (int):  the sampling rate of the audio signal
        samples (np.array): array of audio samples

    Returns:
        str:    filepath of the saved wav file
    """
    mono_samples = np.mean(samples, axis=1) if samples.ndim > 1 else samples
    num_of_resamples = int(len(mono_samples) * NEMO_SAMPLE_RATE / sample_rate)
    resamples = resample(mono_samples, num_of_resamples)
    filepath = os.path.join(AUDIO_STORAGE_DIR, f"{uuid.uuid4().hex}.wav")
    wavfile.write(filepath, NEMO_SAMPLE_RATE, resamples)
    return filepath


def _get_audio_transcription(filepath):
    """ Extract a text transcription from a wav file

    Parameters:
        filepath (str): path to a wav file

    Returns:
        str:    the text transcription extracted from the wav file
    """
    def transcribe_yes_stdout():
            try:
                results = _model.transcribe([filepath], return_hypotheses=True)
            except IndexError:
                results = []

            return results

    def transcribe_no_stdout():
            sys.stdout = io.StringIO()
            sys.stderr = io.StringIO()

            try:
                results = _model.transcribe([filepath], return_hypotheses=True)
            except IndexError:
                results = []

            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__

            return results

    log_message_template = "speechEvent: transcription results -> {text} (confidence: {conf})"
    hypotheses = transcribe_yes_stdout() if VERBOSE_MODE else transcribe_no_stdout()

    if len(hypotheses) == 0:
        transcription = SPEECH_NOT_RECOGNISED_TEXT
        log_message = log_message_template.format(text=transcription, conf=-1)
    else:
        transcription = hypotheses[0][0].text
        score = round(np.exp(hypotheses[0][0].score), 4)
        log_message = log_message_template.format(text=f"{transcription}", conf=score)
        if score < CONFIDENCE:
            log_message = log_message_template.format(
                text=f"{transcription} [unrecognised]", conf=score
            )
            transcription = SPEECH_NOT_RECOGNISED_TEXT
        if len(transcription.strip()) == 0:
            transcription = SPEECH_NOT_RECOGNISED_TEXT
            log_message = log_message_template.format(text=transcription, conf=-1)

    rospy.loginfo(log_message) if VERBOSE_MODE else "pass"

    return transcription


def _set_transcription_language(language):
    """ Sets the language of operation of speechEvent

    Parameters:
        language:   language speechEvent is to be set to

    Returns:
        int:        1 for success and 0 for failure
    """
    global LANGUAGE, _model

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
        device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    else:
        device = torch.device("cpu")

    LANGUAGE = language.strip().lower()
    _model = nemo_asr.models.EncDecCTCModel.restore_from(
        restore_path=RW_MODEL_PATH if LANGUAGE == "kinyarwanda" else EN_MODEL_PATH,
        map_location=device
    )

    rospy.loginfo(
        f"speechEvent: language set to {LANGUAGE.capitalize()}"
    ) if VERBOSE_MODE else "pass"

    return 1


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
    _streamed_samples = np.concatenate((_streamed_samples, np.array(data.data, dtype=np.float32)))


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
        _streamed_samples = np.array([], dtype=np.float32)
        _last_audio_received_at = None
        return

    if _last_audio_received_at is None:
        return

    currrent_time = time.time()

    if currrent_time - _first_audio_received_at < MAX_UTTERANCE_LENGTH:
        if currrent_time - _last_audio_received_at < SPEECH_PAUSE_PERIOD:
            return

    filepath = _save_audio(SAMPLE_RATE, _streamed_samples)
    transcription = _get_audio_transcription(filepath)
    _publisher.publish(transcription)

    rospy.loginfo(
        "speechEvent: transcription process (plus utterance length) has taken "
        f"{round(time.time() - _first_audio_received_at, 4)} seconds"
    ) if VERBOSE_MODE else "pass"

    _streamed_samples = np.array([], dtype=np.float32)
    _last_audio_received_at = None
    os.remove(filepath)


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
        display_process = multiprocessing.Process(target=_GUI.run)

        def kill(_, __):
            display_process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, kill)
        display_process.start()

    rospy.spin()


def initialise(config, topics, rw_model_path, en_model_path, audio_storage_dir):
    """ Make preparatory initialisations before running a speechEvent ROS node

    Patameters:
        config:             object containing configuration arguments
        topics:             object containing ROS topics to be subscribed to
        rw_model_path:      path to Kinyarwanda ASR model
        en_model_path:      path to English ASR model
        audio_storage_dir:  path to directory to store generated audio files
    """
    global LANGUAGE, VERBOSE_MODE, CUDA, CONFIDENCE, SPEECH_PAUSE_PERIOD, MAX_UTTERANCE_LENGTH, SAMPLE_RATE, HEARTBEAT_MSG_PERIOD
    global RW_MODEL_PATH, EN_MODEL_PATH, AUDIO_STORAGE_DIR
    global SOUND_DETECTION_TOPIC
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

    if not os.path.exists(audio_storage_dir):
        rospy.logwarn(
            "speechEvent: the audio storage directory is absent from the data "
            "directory, it will be created"
        )
        os.mkdir(audio_storage_dir)

    LANGUAGE = config["language"].strip().lower()
    VERBOSE_MODE = True if config["verboseMode"].strip().lower() == "true" else False
    CUDA = True if config["cuda"].strip().lower() == "true" else False
    CONFIDENCE = float(config["confidence"].strip())
    SPEECH_PAUSE_PERIOD = float(config["speechPausePeriod"].strip())
    MAX_UTTERANCE_LENGTH = float(config["maxUtteranceLength"].strip())
    SAMPLE_RATE = int(config["sampleRate"].strip())
    HEARTBEAT_MSG_PERIOD = int(config["heartbeatMsgPeriod"].strip())
    SOUND_DETECTION_TOPIC = topics["soundDetection"].strip()
    RW_MODEL_PATH = rw_model_path
    EN_MODEL_PATH = en_model_path
    AUDIO_STORAGE_DIR = audio_storage_dir

    nemo_logging.Logger().remove_stream_handlers()
    _set_transcription_language(LANGUAGE)
    _publisher = rospy.Publisher(PUB_TOPIC, String, queue_size=10)
