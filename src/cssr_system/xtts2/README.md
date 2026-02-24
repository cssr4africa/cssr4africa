# XTTS2 Streaming TTS to NAO/Pepper Robot

## Overview

This ROS package provides a Text-to-Speech (TTS) action server that generates speech using Coqui XTTS v2 and streams it to NAO or Pepper robots. The system uses real-time audio generation with chunked streaming for low-latency speech playback.

## Architecture

### Components

1. **TTS Action Server** (`streaming_tts_to_nao.py`)
   - ROS Action Server exposing the `tts_action` interface
   - Uses RealtimeTTS with Coqui XTTS v2 engine
   - Streams audio chunks to NAO/Pepper via Python 2 bridge script

2. **Audio Processing Pipeline**
   - Text → XTTS v2 Engine → Audio Chunks → NAO Robot
   - Concurrent audio generation and streaming
   - Configurable chunk duration for streaming optimization

## Prerequisites

### Dependencies

- **ROS** (tested with ROS Noetic)
- **Python 3.10+** (for the action server)
- **Python 2.7** (for NAO/Pepper communication via NAOqi)
- **RealtimeTTS**: `pip install RealtimeTTS`
- **NAOqi Python SDK** (Python 2.7 bindings)

### Required ROS Messages

The package uses custom action messages defined in `xtts2/msg/`:

```
english_stream_ttsAction.action
```

Action definition structure:
```
# Goal
string text
---
# Result
bool success
string message
---
# Feedback
string status
```

## Configuration

### ROS Parameters

Configure the action server using ROS parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~nao_ip` | string | "172.29.111.230" | NAO/Pepper robot IP address |
| `~nao_port` | int | 9559 | NAO/Pepper NAOqi port |
| `~nao_password` | string | "nao" | Robot authentication password |
| `~chunk_duration` | float | 15.0 | Duration of each audio chunk in seconds |

### Launch File Example

```xml
<launch>
  <node pkg="xtts2" type="streaming_tts_to_nao.py" name="tts_action_server" output="screen">
    <param name="nao_ip" value="192.168.1.100" />
    <param name="nao_port" value="9559" />
    <param name="nao_password" value="nao" />
    <param name="chunk_duration" value="10.0" />
  </node>
</launch>
```

## Interfacing with the Action Server

### From Python (Action Client)

#### Simple Example

```python
#!/usr/bin/env python3
import rospy
import actionlib
from xtts2.msg import english_stream_ttsAction, english_stream_ttsGoal

def speak_text(text):
    # Create action client
    client = actionlib.SimpleActionClient('tts_action', english_stream_ttsAction)
    
    # Wait for server to start
    rospy.loginfo("Waiting for TTS action server...")
    client.wait_for_server()
    
    # Create and send goal
    goal = english_stream_ttsGoal()
    goal.text = text
    
    # Send goal and wait for result
    client.send_goal(goal)
    client.wait_for_result()
    
    # Get result
    result = client.get_result()
    return result.success, result.message

if __name__ == '__main__':
    rospy.init_node('tts_client_example')
    
    success, message = speak_text("Hello, I am a NAO robot speaking with streaming TTS!")
    
    if success:
        rospy.loginfo(f"Speech succeeded: {message}")
    else:
        rospy.logerr(f"Speech failed: {message}")
```

#### Advanced Example with Feedback

```python
#!/usr/bin/env python3
import rospy
import actionlib
from xtts2.msg import english_stream_ttsAction, english_stream_ttsGoal

class TTSClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('tts_action', english_stream_ttsAction)
        rospy.loginfo("Waiting for TTS action server...")
        self.client.wait_for_server()
        rospy.loginfo("TTS action server connected!")
    
    def feedback_cb(self, feedback):
        """Called when feedback is received from the server"""
        rospy.loginfo(f"TTS Feedback: {feedback.status}")
    
    def speak(self, text, wait=True):
        """Send text to be spoken"""
        goal = english_stream_ttsGoal()
        goal.text = text
        
        # Send goal with feedback callback
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        
        if wait:
            # Wait for result
            self.client.wait_for_result()
            result = self.client.get_result()
            state = self.client.get_state()
            
            return result.success, result.message, state
        else:
            # Non-blocking
            return None, None, None
    
    def cancel(self):
        """Cancel current speech"""
        self.client.cancel_goal()
    
    def is_speaking(self):
        """Check if currently speaking"""
        state = self.client.get_state()
        return state == actionlib.GoalStatus.ACTIVE

if __name__ == '__main__':
    rospy.init_node('advanced_tts_client')
    
    tts = TTSClient()
    
    # Example 1: Speak and wait
    success, message, state = tts.speak("This is a test of the streaming TTS system.")
    rospy.loginfo(f"Result: success={success}, message={message}")
    
    # Example 2: Non-blocking speech
    tts.speak("This is non-blocking speech.", wait=False)
    
    # Do other work...
    while tts.is_speaking():
        rospy.loginfo("Still speaking...")
        rospy.sleep(1.0)
    
    # Example 3: Cancel speech
    tts.speak("This is a very long sentence that will be cancelled.", wait=False)
    rospy.sleep(2.0)
    tts.cancel()
    rospy.loginfo("Speech cancelled")
```

### From Command Line (rostopic/rosaction)

#### Send a goal using actionlib command-line tools:

```bash
# Or use rosaction
rosaction call /tts_action "text: 'Hello from command line'"
```

#### Monitor feedback:

```bash
rostopic echo /tts_action/feedback
```

#### Check server status:

```bash
rostopic list | grep tts_action
# Should show:
# /tts_action/cancel
# /tts_action/feedback
# /tts_action/goal
# /tts_action/result
# /tts_action/status
```

## System Behavior

### Normal Operation Flow

1. Client sends goal with text to `tts_action`
2. Server receives goal and initializes generation
3. Two parallel threads start:
   - **Generation Thread**: Produces audio chunks using XTTS v2
   - **Streaming Thread**: Sends chunks to NAO as they become available
4. Server publishes feedback with streaming status
5. When complete, server returns result (success/failure)

### Error Handling

- **Empty Text**: Goal aborted with error message
- **Generation Error**: Logged, result set to failure
- **NAO Communication Error**: Logged as warning, continues with next chunk
- **Preemption**: Stops generation, cleans up, returns preempted status

### Streaming Details

- **Chunk Duration**: Configurable (default 15 seconds)
- **Sample Rate**: 24000 Hz (16-bit mono)
- **Format**: WAV files sent via Python 2 bridge
- **Cleanup**: Temporary files deleted after playback

## Troubleshooting

### Server Not Starting

```bash
# Check if server is running
rosnode list | grep tts_action_server

# Check server logs
rosnode info /tts_action_server
```

### No Audio on Robot

1. Verify NAO/Pepper IP and connectivity:
   ```bash
   ping <nao_ip>
   ```

2. Check Python 2 script exists:
   ```bash
   rospack find xtts2
   ls $(rospack find xtts2)/scripts/send_and_play_audio.py
   ```

3. Test NAOqi connection separately

### Action Client Timeout

```python
# Increase timeout for long speech
client.wait_for_result(timeout=rospy.Duration(120.0))
```

## Performance Tuning

### Adjust Chunk Duration

Smaller chunks = lower latency, more overhead 
Larger chunks = higher latency, less overhead 

```xml
<param name="chunk_duration" value="5.0" />  <!-- 5 seconds for low latency -->
```

### Voice Cloning

Place custom voice samples in:
```
<package_path>/voice_clones/female01.wav
```

The system will clone this voice for speech generation.

## API Reference

### Action Interface: `tts_action`

**Type**: `xtts2/english_stream_ttsAction`

#### Goal
- `string text` - Text to be spoken

#### Result
- `bool success` - Whether speech completed successfully
- `string message` - Status message or error description

#### Feedback
- `string status` - Current streaming status (e.g., "Streaming chunk 3...")

### State Values

Use `actionlib.GoalStatus` constants:
- `PENDING` = 0
- `ACTIVE` = 1
- `PREEMPTED` = 2
- `SUCCEEDED` = 3
- `ABORTED` = 4
- `REJECTED` = 5
- `LOST` = 8

