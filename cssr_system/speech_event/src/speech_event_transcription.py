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
    mp_streamed_samples, mp_tensor_lock, mp_misc_lock, mp_current_idx, mp_lang,
    mp_log_lock, mp_log_level, mp_log_message, mp_pub_pipe, cuda, rw_model_path,
    en_model_path, sample_rate, min_utterance_len, audio_max_len, conf, verbose,
    model_name, inter_utterance_len, vad_model_path, vad_threshold
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
    vad_frame_size = sample_rate
    voice_activity_detected = False
    no_voice_count = 0
    max_no_voice_count = (inter_utterance_len // vad_frame_size) + 1

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
            num_of_samples = mp_current_idx.value - current_idx
            if num_of_samples < 0:
                num_of_samples = audio_max_len - current_idx + mp_current_idx.value

        if num_of_samples == 0 or prev_num_of_samples == num_of_samples:
            prev_num_of_samples = num_of_samples
            time.sleep(inter_utterance_len)
            if not voice_activity_detected:
                continue
            else:
                with mp_tensor_lock:
                    num_of_samples = mp_current_idx.value - current_idx
                    if num_of_samples < 0:
                        num_of_samples = audio_max_len - current_idx + mp_current_idx.value
                if prev_num_of_samples != num_of_samples:
                    prev_num_of_samples = num_of_samples
                    continue
        else:
            prev_num_of_samples = num_of_samples

            if num_of_samples < min_utterance_len:
                time.sleep(inter_utterance_len)
                continue

            if voice_activity_detected:
                continue

            last_idx = current_idx + num_of_samples
            if last_idx >= audio_max_len:
                last_idx -= audio_max_len

            if voice_activity_detected:
                if last_idx >= vad_frame_size:
                    audio = mp_streamed_samples[last_idx - vad_frame_size: last_idx].clone()
                else:
                    audio = mp_streamed_samples[0: last_idx].clone()
            else:
                if last_idx >= current_idx:
                    audio = mp_streamed_samples[current_idx: last_idx].clone()
                else:
                    audio = mp_streamed_samples[0: last_idx].clone()

            if audio.shape[0] == 0:
                continue

            resampled_audio = resampler(audio)

            if len(silero_vad.get_speech_timestamps(resampled_audio, vad, threshold=vad_threshold)) > 0:
                voice_activity_detected = True
                no_voice_count = 0
            else:
                no_voice_count += 1

            if no_voice_count < max_no_voice_count:
                continue

        no_voice_count = 0
        start_time = time.time()

        if current_idx + num_of_samples < audio_max_len:
            audio_tensor = mp_streamed_samples[current_idx:current_idx+num_of_samples].clone()
        else:
            audio_tensor = torch.cat((
                mp_streamed_samples[current_idx:audio_max_len].clone(), mp_streamed_samples[0:audio_max_len-current_idx+num_of_samples].clone()
            ))

        resampled = resampler(audio_tensor)

        if voice_activity_detected:
            normalised = resampled / resampled.abs().max()
            hypotheses = se_utils.get_hypotheses(MODEL_NAME, model, normalised)
            transcription, log_message = se_utils.get_transcription(
                MODEL_NAME, hypotheses, SPEECH_NOT_RECOGNISED_TEXT, conf
            )

            with mp_log_lock:
                mp_log_level.value = LOG_LEVELS["info"] if verbose else LOG_LEVELS["none"]
                mp_log_message.value = f"{log_message} - {round(time.time() - start_time, 4)} seconds".encode("UTF-8")

            mp_pub_pipe.send(transcription) if transcription != SPEECH_NOT_RECOGNISED_TEXT else "pass"

        current_idx += num_of_samples
        voice_activity_detected = False

        if current_idx >= audio_max_len:
            current_idx -= audio_max_len
