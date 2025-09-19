import copy
import time

import nemo.collections.asr as nemo_asr
import nemo.utils.nemo_logging as nemo_logging
import pyaudio
import torch


nemo_logging.Logger().set_verbosity(nemo_logging.Logger.ERROR)

MODEL_PATH = "./cssr_system/speech_event/models/stt_en_fastconformer_hybrid_large_streaming_multi.nemo"
READ_SR = 16000
READ_CHUNK_SIZE = int(0.2 * READ_SR)  # 0.2 seconds
READ_FORMAT = pyaudio.paFloat32
READ_NUM_OF_CHANNELS = 1
TRANSCRIBE_CHUNK_SIZE = int(1 * READ_SR)  # 1 second
TRANSCRIBE_STRIDE = TRANSCRIBE_CHUNK_SIZE  # 1 second
AUDIO_MAX_LEN = READ_SR * 60  # 60 seconds


def transcribe(audios, audios_len, lock):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    asr_model = nemo_asr.models.EncDecHybridRNNTCTCBPEModel.restore_from(restore_path=MODEL_PATH, map_location=device)
    asr_model.eval()
    asr_model.encoder.set_default_att_context_size([70, 13])
    asr_model.change_decoding_strategy(decoder_type="ctc")

    initial_cache_last_channel, initial_cache_last_time, initial_cache_last_channel_len = asr_model.encoder.get_initial_cache_state(batch_size=1)
    cache_last_channel = copy.deepcopy(initial_cache_last_channel)
    cache_last_time = copy.deepcopy(initial_cache_last_time)
    cache_last_channel_len = copy.deepcopy(initial_cache_last_channel_len)

    pred_out_stream = None
    hypotheses = None
    current_idx = 0

    print("Listening ...")

    while True:
        try:
            with lock:
                num_of_samples = audios_len.value

            if num_of_samples < TRANSCRIBE_CHUNK_SIZE:
                time.sleep(0.5)
                continue

            with lock:
                audio_tensor = audios[current_idx:current_idx+TRANSCRIBE_CHUNK_SIZE]

            audio_tensor = audio_tensor.unsqueeze(0)
            audio_tensor = (audio_tensor / audio_tensor.abs().max()).to(device)
            audio_length = torch.tensor([audio_tensor.shape[1]], dtype=torch.int64).to(device)

            features, length = asr_model.preprocessor(input_signal=audio_tensor, length=audio_length)

            with torch.no_grad():
                (
                    pred_out_stream,
                    transcribed_texts,
                    cache_last_channel,
                    cache_last_time,
                    cache_last_channel_len,
                    hypotheses
                ) = asr_model.conformer_stream_step(
                    processed_signal=features,
                    processed_signal_length=length,
                    cache_last_channel=cache_last_channel,
                    cache_last_time=cache_last_time,
                    cache_last_channel_len=cache_last_channel_len,
                    keep_all_outputs=False,
                    previous_hypotheses=hypotheses,
                    previous_pred_out=pred_out_stream,
                    drop_extra_pre_encoded=None,
                    return_transcription=True
                )

            transcription = transcribed_texts[0].text
            score = round(torch.exp(torch.tensor(transcribed_texts[0].score)).item(), 4)
            current_idx += TRANSCRIBE_STRIDE

            if current_idx >= AUDIO_MAX_LEN:
                current_idx = 0

            with lock:
                audios_len.value -= TRANSCRIBE_CHUNK_SIZE

            print(f"{transcription} | {score}")

        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    torch.multiprocessing.set_start_method("spawn")

    audios = torch.zeros(AUDIO_MAX_LEN, dtype=torch.float32).share_memory_()
    audios_len = torch.multiprocessing.Value("i", 0)
    lock = torch.multiprocessing.Lock()
    current_idx = 0

    p = pyaudio.PyAudio()
    stream = p.open(
        format=READ_FORMAT,
        channels=READ_NUM_OF_CHANNELS,
        rate=READ_SR,
        input=True,
        frames_per_buffer=READ_CHUNK_SIZE
    )

    transcribe_process = torch.multiprocessing.Process(target=transcribe, args=(audios, audios_len, lock))
    transcribe_process.start()

    try:
        while True:
            audio_chunk = stream.read(READ_CHUNK_SIZE, exception_on_overflow=False)
            audio_tensor = torch.frombuffer(bytearray(audio_chunk), dtype=torch.float32)

            with lock:
                audios[current_idx:current_idx+READ_CHUNK_SIZE] = audio_tensor
                current_idx += READ_CHUNK_SIZE
                audios_len.value += READ_CHUNK_SIZE

            if current_idx > AUDIO_MAX_LEN - READ_CHUNK_SIZE:
                current_idx = 0
    except KeyboardInterrupt:
        print("Stopping ...")
    finally:
        transcribe_process.kill()
        stream.stop_stream()
        stream.close()
        p.terminate()
