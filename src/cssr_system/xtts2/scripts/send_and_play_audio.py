#!/usr/bin/python2
from naoqi import ALProxy
import subprocess
import sys
import time

audio_path = sys.argv[1]
audio_name = audio_path.split("/")[-1]
IP = "172.29.111.230"
PORT = 9559

print "Local audio file:", audio_path
print "Audio file name:", audio_name

# Transfer file to NAO robot
print "Transferring audio file to NAO..."
result = subprocess.call([
    "sshpass", "-p", "nao", "scp",
    "-o", "StrictHostKeyChecking=no",
    audio_path, 
    "nao@" + IP + ":/home/nao/" + audio_name
])

if result != 0:
    print "Error: File transfer failed with exit code:", result
    sys.exit(1)

print "File transferred successfully"

# Small delay to ensure filesystem sync
time.sleep(0.2)

# Verify file exists on NAO
print "Verifying file on NAO..."
verify_result = subprocess.call([
    "sshpass", "-p", "nao", "ssh",
    "-o", "StrictHostKeyChecking=no",
    "nao@" + IP,
    "test -f /home/nao/" + audio_name
])

if verify_result != 0:
    print "Error: File not found on NAO after transfer!"
    sys.exit(1)

print "File verified on NAO"
print "Initializing the audio player"

try:
    audio_player = ALProxy("ALAudioPlayer", IP, PORT)
    # IMPORTANT: Use the path on the NAO robot, not the local path
    remote_file_path = "/home/nao/" + audio_name
    print "Playing file:", remote_file_path
    # audio_player.playFile(remote_file_path)
    taskid = audio_player.post.playFile(remote_file_path)  # Non-blocking call
    # Wait for playback to finish
    while audio_player.isRunning(taskid):
        time.sleep(0.1)
    # audio_player.post.playFile(remote_file_path) # Non-blocking call
    print "Audio playback completed"
except Exception as e:
    print "Error playing audio:", str(e)
    sys.exit(1)

print "Cleaning up..."
# Clean up the file on NAO
subprocess.call([
    "sshpass", "-p", "nao", "ssh",
    "-o", "StrictHostKeyChecking=no",
    "nao@" + IP, 
    "rm -f /home/nao/" + audio_name
])

print "Done"
            # Local playback using aplay (Linux) or ffplay (cross-platform)
            # subprocess.run(["aplay", audio_file])   