"""
speech_event_utils.py - utility functions for the speechEvent ROS node

Author:     Clifford Onyonka
Date:       2025-07-11
Version:    v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import nemo.collections.asr as nemo_asr
import nemo.utils.nemo_logging as nemo_logging
import torch
import whisper


nemo_logging.Logger().set_verbosity(nemo_logging.Logger.ERROR)


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


def get_model(model_name, rw_model_path, en_model_path, lang, device):
    if model_name == "conformer-transducer":
        model = nemo_asr.models.EncDecRNNTBPEModel.restore_from(
            restore_path=rw_model_path if lang == "kinyarwanda" else en_model_path,
            map_location=device
        )
    elif model_name == "parakeet":
        model = nemo_asr.models.EncDecHybridRNNTCTCBPEModel.restore_from(
            restore_path=rw_model_path if lang == "kinyarwanda" else en_model_path,
            map_location=device
        )
    elif model_name == "whisper":
        model = whisper.load_model(rw_model_path if lang == "kinyarwanda" else en_model_path, device=device)
    else:
        raise ValueError(f"'{model_name}' not supported")

    model.eval()

    return model


def get_hypotheses(model_name, model, audio_samples):
    with torch.no_grad():
        if model_name == "conformer-transducer":
            hypotheses = model.transcribe(audio_samples, return_hypotheses=True)
        elif model_name == "parakeet":
            hypotheses = model.transcribe(audio_samples, return_hypotheses=True)
        elif model_name == "whisper":
            hypotheses = model.transcribe(audio_samples)
        else:
            raise ValueError(f"'{model_name}' not supported")

    return hypotheses


def get_transcription(model_name, hypotheses, text_for_unrecognised, conf_threshold):
    log_message_template = "speechEvent: transcription results -> {text} (confidence: {conf})"
    hypotheses_len = None

    if model_name == "conformer-transducer":
        hypotheses_len = len(hypotheses)
    elif model_name == "parakeet":
        hypotheses_len = len(hypotheses)
    elif model_name == "whisper":
        hypotheses_len = len(hypotheses["segments"])
    else:
        raise ValueError(f"'{model_name}' not supported")

    if hypotheses_len == 0:
        transcription = text_for_unrecognised
        log_message = log_message_template.format(text=transcription, conf=-1)
    else:
        if model_name == "conformer-transducer":
            transcription = hypotheses[0].text
            score = round(torch.exp(torch.tensor(hypotheses[0].score)).item(), 4)
        elif model_name == "parakeet":
            transcription = hypotheses[0].text
            score = round(torch.exp(torch.tensor(hypotheses[0].score)).item(), 4)
        elif model_name == "whisper":
            transcription = hypotheses["text"]
            score = 1 - round(hypotheses["segments"][0]["no_speech_prob"], 4)
        else:
            transcription = ""
            score = -1

        log_message = log_message_template.format(text=f"{transcription}", conf=score)
        if score < conf_threshold:
            log_message = log_message_template.format(
                text=f"{transcription} [unrecognised]", conf=score
            )
            transcription = text_for_unrecognised
    
    return transcription, log_message
