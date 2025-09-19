import time

import numpy as np
import textdistance
import torch
import faster_whisper
from pydub import AudioSegment


model = faster_whisper.WhisperModel("small")


def chunk_audio(audio, chunk_length_ms, overlap_ms):
    """
    Split audio (pydub.AudioSegment) into chunks with overlap.
    """
    step = chunk_length_ms - overlap_ms
    chunks = []
    for start in range(0, len(audio), step):
        end = start + chunk_length_ms
        chunk = audio[start:end]
        if len(chunk) < chunk_length_ms:
            chunk += AudioSegment.silent(duration=(chunk_length_ms - len(chunk)))
        chunks.append(chunk)
        if end >= len(audio):
            break
    return chunks


def clean_overlap_text(prev_text, curr_text, similarity_score=0.5):
    max_sim_chars = min(len(prev_text), len(curr_text))

    for i in range(max_sim_chars, 0, -1):
        if textdistance.hamming.normalized_similarity(prev_text[-i:], curr_text[:i]) >= similarity_score:
            return prev_text[:-i] + curr_text

    return prev_text + curr_text


def transcribe_streaming(audio_path, chunk_length_sec=5, overlap_sec=0.5):
    audio = AudioSegment.from_file(audio_path)
    chunk_length_ms = int(chunk_length_sec * 1000)
    overlap_ms = int(overlap_sec * 1000)
    chunks = chunk_audio(audio, chunk_length_ms, overlap_ms)
    final_transcript = ""

    start = time.time()
    for i, chunk in enumerate(chunks):
        print(f"Processing chunk {i + 1} / {len(chunks)}...")
        samples = torch.tensor(chunk.get_array_of_samples(), dtype=torch.float32) / 32768.0

        if chunk.channels > 1:
            samples = samples.reshape((-1, chunk.channels))
            samples = samples.mean(axis=1)

        segments, _ = model.transcribe(samples.numpy().astype(np.float32), language="en")
        for segment in segments:
            final_transcript = clean_overlap_text(final_transcript, segment.text)

    print(f"Time taken: {time.time() - start} seconds")

    return final_transcript


if __name__ == "__main__":
    audio_file = "/home/user/Desktop/speech.wav"
    transcript = transcribe_streaming(audio_file)
    print("\n--- Final Transcript ---\n")
    print(transcript)
