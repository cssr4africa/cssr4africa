import signal
import time

import silero_vad
import torch
import torchaudio

import speech_event_utils as se_utils


ASR_SAMPLE_RATE = 16000  # of audio required by ASR
SPEECH_NOT_RECOGNISED_TEXT = "Error: speech not recognized"
LOG_LEVELS = {
    "none": -1,
    "debug": 0,
    "info": 1,
    "warning": 2,
    "error": 3,
    "critical": 4
}
MODEL_NAME = None

model = None
lang = None


def set_model(received_lang, cuda, rw_model_path, en_model_path):
    global model, lang

    lang = received_lang

    if cuda:
        if not torch.cuda.is_available():
            print("speechEvent: CUDA not available, defaulting to CPU")
        device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    else:
        device = torch.device("cpu")

    model = se_utils.get_model(MODEL_NAME, rw_model_path, en_model_path, lang, device)


def run_transcriptions(
    mp_streamed_samples, mp_tensor_lock, mp_misc_lock, mp_samples_len, mp_lang,
    mp_log_lock, mp_log_level, mp_log_message, mp_pub_lock, mp_pub_transcription,
    cuda, rw_model_path, en_model_path, sample_rate, audio_max_len, conf, verbose,
    model_name, inter_utterance_len, vad_model_path, vad_threshold, vad_min_speech_duration
):
    global MODEL_NAME

    MODEL_NAME = model_name
    current_idx = 0
    prev_num_of_samples = 0
    to_exit = False
    resampler = torchaudio.transforms.Resample(orig_freq=sample_rate, new_freq=ASR_SAMPLE_RATE)
    device = torch.device("cpu") if not cuda else (
        torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    )
    vad = silero_vad.utils_vad.init_jit_model(vad_model_path, device)

    def set_to_exit(_, __):
        nonlocal to_exit
        to_exit = True

    signal.signal(signal.SIGINT, set_to_exit)
    signal.signal(signal.SIGTERM, set_to_exit)

    while not to_exit:
        with mp_misc_lock:
            received_lang = mp_lang.value.decode("UTF-8")

        if lang != received_lang:
            set_model(received_lang, cuda, rw_model_path, en_model_path)

        with mp_tensor_lock:
            num_of_samples = mp_samples_len.value

        if num_of_samples == 0:
            time.sleep(inter_utterance_len)
            continue

        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        # Checking for pause here, maybe check for no VAD somewhere above or below here
        if prev_num_of_samples != num_of_samples:  # Checking for pause here, maybe check for no VAD somewhere above or below here
            prev_num_of_samples = num_of_samples
            time.sleep(inter_utterance_len)
            continue

        start_time = time.time()
        with mp_tensor_lock:
            audio_tensor = mp_streamed_samples[current_idx:current_idx+num_of_samples].clone()

        resampled = resampler(audio_tensor)

        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription (if a pause is detected, or no VAD detected)
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        ############## Run VAD on last inter_utterance_len samples, if they are noise then do transcription
        if len(silero_vad.get_speech_timestamps(
            resampled, vad, threshold=vad_threshold, min_speech_duration_ms=vad_min_speech_duration
        )) > 0:
            normalised = resampled / resampled.abs().max()
            hypotheses = se_utils.get_hypotheses(MODEL_NAME, model, normalised)
            transcription, log_message = se_utils.get_transcription(
                MODEL_NAME, hypotheses, SPEECH_NOT_RECOGNISED_TEXT, conf
            )

            with mp_log_lock:
                mp_log_level.value = LOG_LEVELS["info"] if verbose else LOG_LEVELS["none"]
                mp_log_message.value = f"{log_message} - {round(time.time() - start_time, 4)} seconds".encode("UTF-8")

            with mp_pub_lock:
                mp_pub_transcription.value = transcription.encode("UTF-8") if transcription != SPEECH_NOT_RECOGNISED_TEXT else "".encode("UTF-8")

        current_idx += num_of_samples

        if current_idx >= audio_max_len:
            current_idx = 0

        with mp_tensor_lock:
            mp_samples_len.value -= num_of_samples
