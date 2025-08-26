import time

import nemo.utils.nemo_logging as nemo_logging
import torch
import torchaudio

import speech_event_utils as se_utils


SAMPLE_RATE = 16000  # of audio required for ASR
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

nemo_logging.Logger().set_verbosity(nemo_logging.Logger.ERROR)


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
    model_name
):
    global MODEL_NAME

    MODEL_NAME = model_name
    resampler = None
    current_idx = 0
    prev_num_of_samples = 0

    while True:
        with mp_misc_lock:
            received_lang = mp_lang.value.decode("UTF-8")

        if lang != received_lang:
            set_model(received_lang, cuda, rw_model_path, en_model_path)

        with mp_tensor_lock:
            num_of_samples = mp_samples_len.value

        if num_of_samples == 0:
            time.sleep(0.5)
            continue

        if prev_num_of_samples != num_of_samples:
            prev_num_of_samples = num_of_samples
            time.sleep(0.5)
            continue

        start_time = time.time()
        with mp_tensor_lock:
            audio_tensor = mp_streamed_samples[current_idx:current_idx+num_of_samples].clone()

        if resampler is None:
            resampler = torchaudio.transforms.Resample(orig_freq=sample_rate, new_freq=SAMPLE_RATE)

        normalised = audio_tensor / audio_tensor.abs().max()
        resampled = resampler(normalised).to(model.device)

        hypotheses =se_utils.get_hypotheses(MODEL_NAME, model, resampled)
        transcription, log_message = se_utils.get_transcription(
            MODEL_NAME, hypotheses, SPEECH_NOT_RECOGNISED_TEXT, conf
        )

        current_idx += num_of_samples

        if current_idx >= audio_max_len:
            current_idx = 0

        with mp_tensor_lock:
            mp_samples_len.value -= num_of_samples

        with mp_log_lock:
            mp_log_level.value = LOG_LEVELS["info"] if verbose else LOG_LEVELS["none"]
            mp_log_message.value = f"{log_message} - {round(time.time() - start_time, 4)} seconds".encode("UTF-8")

        with mp_pub_lock:
            mp_pub_transcription.value = transcription.encode("UTF-8") if transcription != SPEECH_NOT_RECOGNISED_TEXT else "".encode("UTF-8")
