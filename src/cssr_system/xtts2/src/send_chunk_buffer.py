#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import struct
import numpy as np
from naoqi import ALProxy

def read_exactly(n):
    """Read exactly n bytes from stdin"""
    data = b""
    while len(data) < n:
        chunk = sys.stdin.read(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def main():
    if len(sys.argv) < 4:
        print("Usage: tts_audio_bridge.py <ip> <port> <sample_rate>")
        sys.exit(1)

    ip = sys.argv[1]
    port = int(sys.argv[2])
    sample_rate = int(sys.argv[3])

    audio_dev = ALProxy("ALAudioDevice", ip, port)

    sys.stderr.write("[Bridge] Connected to Pepper audio at {}:{}\n".format(ip, port))
    sys.stderr.flush()

    while True:
        # Read 4-byte size header
        size_data = read_exactly(4)
        if not size_data:
            break
        (chunk_size,) = struct.unpack("<I", size_data)

        # Read PCM chunk
        pcm_data = read_exactly(chunk_size)
        if not pcm_data:
            break

        # Convert PCM16 → float32 [-1, 1]
        samples = np.frombuffer(pcm_data, dtype=np.int16).astype(np.float32) / 32768.0
        samples_list = samples.tolist()

        # Send to Pepper speakers (mono = 1).
        audio_dev.sendLocalBufferToOutput(samples_list, 1, sample_rate)

if __name__ == "__main__":
    main()
