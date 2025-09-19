import time

import pyaudio
import torch
import whisper


MODEL_PATH = "cssr_system/speech_event/models/whisper_small_en.pt"
READ_SR = 16000
READ_CHUNK_SIZE = int(0.2 * READ_SR)  # 0.2 seconds
READ_FORMAT = pyaudio.paFloat32
READ_NUM_OF_CHANNELS = 1
TRANSCRIBE_CHUNK_SIZE = int(1 * READ_SR)  # 1 second
TRANSCRIBE_STRIDE = int(0.5 * READ_SR)  # 0.5 second
AUDIO_MAX_LEN = READ_SR * 60  # 60 seconds


def clean_overlap_text(prev_text, curr_text):
    max_overlap = min(len(prev_text), len(curr_text))
    for i in range(max_overlap, 0, -1):
        if prev_text[-i:] == curr_text[:i]:
            return prev_text + curr_text[i:]
    return prev_text + curr_text


def transcribe(audios, audios_len, lock):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    asr_model = whisper.load_model(MODEL_PATH).to(device)
    asr_model.eval()

    current_idx = 0
    prev_tokens = None
    transcription = ""

    print("Listening ...")

    while True:
        try:
            with lock:
                num_of_samples = audios_len.value

            if num_of_samples < TRANSCRIBE_CHUNK_SIZE:
                time.sleep(0.5)
                continue

            with lock:
                audio_tensor = audios[current_idx:current_idx+TRANSCRIBE_CHUNK_SIZE].clone()

            audio_tensor = (audio_tensor / audio_tensor.abs().max()).to(device)

            if prev_tokens is not None:
                decode_options = whisper.DecodingOptions(prompt=prev_tokens)
            else:
                decode_options = whisper.DecodingOptions()

            audio_input = whisper.pad_or_trim(audio_tensor)
            mel = whisper.log_mel_spectrogram(audio_input).to(device)
            results = asr_model.decode(mel, decode_options)
            prev_tokens = results.tokens[-20:]
            transcription = clean_overlap_text(transcription, results.text)
            score = 1 - round(results.no_speech_prob, 4)

            print(f"{transcription} | {score}")

            current_idx += TRANSCRIBE_STRIDE

            if current_idx >= AUDIO_MAX_LEN:
                current_idx = 0

            with lock:
                audios_len.value -= TRANSCRIBE_CHUNK_SIZE

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
        print("\rStopping ...")
    finally:
        transcribe_process.kill()
        stream.stop_stream()
        stream.close()
        p.terminate()
