
<div align="center">
  <h1>Integrated Text-to-Speech</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `textToSpeech` ROS node provides multilingual text-to-speech functionality for the Pepper robot within the CSSR4Africa project. The node supports two languages and two neural TTS backends:

- **English** — [XTTS v2](https://huggingface.co/coqui/XTTS-v2) by Coqui AI, a state-of-the-art multilingual voice-cloning model.
- **Kinyarwanda** — A fine-tuned [YourTTS](https://github.com/coqui-ai/TTS) model trained specifically for Kinyarwanda .

The communication interface (ROS service or ROS action server) and audio playback destination (robot or local machine) are both configurable at runtime via the configuration file. The node exposes its interface at `/textToSpeech/say_text`.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the TTS system architecture, neural models, and implementation details. The deliverable report can be found in [D5.5.2.4 Text-to-Speech](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf).

# Run the Text-to-Speech Node
## Physical Robot 
### Steps
1. **Install the required software components:**

   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_rob_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && catkin_make && source devel/setup.bash
       ```

3. **Setup TTS Environment:**

   The TTS node requires a dedicated Python virtual environment with Python 3.10:

   ```bash
   # Install Python 3.10 if not already installed
   sudo apt update
   sudo apt install python3.10 python3.10-venv python3.10-dev

   # Create a new virtual environment with Python 3.10
   python3.10 -m venv ~/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_text_to_speech_env

   # Activate the virtual environment
   source ~/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_text_to_speech_env/bin/activate

   # Install required Python packages
   pip install -r $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/text_to_speech_requirements.txt

   # Configure ROS path in virtual environment
   echo "import sys; sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')" > $(python -c "import site; print(site.getsitepackages()[0])")/sitecustomize.py

   # Install sshpass for robot communication
   sudo apt-get install sshpass
   ```

4. **Download TTS Model Files:**

   The node requires two sets of model files: the Kinyarwanda YourTTS model and the English XTTS v2 model.

   **Kinyarwanda YourTTS model:**

   ```bash
   # If git-lfs is already set up, skip the install step
   sudo apt-get update && sudo apt-get install git-lfs
   cd && git lfs install

   # Clone the models from HuggingFace:
   git clone https://huggingface.co/cssr4africa/cssr4africa_models

   # Copy the Kinyarwanda model files to the model/ directory:
   mkdir -p ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model
   unzip cssr4africa_models/text_to_speech/model.zip -d ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech
   ```

   Verify the Kinyarwanda model files are in place:

   ```bash
   ls ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model
   # Expected: model.pth  config.json  speakers.pth  SE_checkpoint.pth.tar  config_se.json  conditioning_audio.wav
   ```

   **English XTTS v2 model:**

   ```bash
   # Download the XTTS v2 model (requires ~2 GB of disk space)
   pip install huggingface_hub
   python3 -c "
   from huggingface_hub import snapshot_download
   snapshot_download(repo_id='coqui/XTTS-v2', local_dir=os.path.expanduser('~/models/v2.0.2'))
   "
   ```

   Alternatively, download manually from https://huggingface.co/coqui/XTTS-v2 and place the files under `~/models/v2.0.2/`.

   Expected contents of `~/models/v2.0.2/`:
   ```
   config.json   model.pth   vocab.json   speakers_xtts.pth   dvae.pth
   ```

5. **Update Configuration File:**
   
   Navigate to the configuration file located at:
   `~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/config/text_to_speech_configuration.ini`

   Update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `language` | Default synthesis language when the request does not specify one | `english` or `kinyarwanda` |
   | `interface` | ROS communication interface to expose | `service`, `action`, or `both` |
   | `playback_mode` | Where synthesized audio is played | `naoqi` (robot) or `local` (this machine) |
   | `verboseMode` | Enable detailed logging | `True` or `False` |
   | `ip` | Pepper robot IP address (used when `playback_mode = naoqi`) | e.g., `172.29.111.240` |
   | `port` | Robot communication port (used when `playback_mode = naoqi`) | `9559` (default) |
   | `useCuda` | Enable GPU acceleration for synthesis | `True` or `False` |
   | `kinyarwandaModelPath` | Path to the Kinyarwanda YourTTS model directory | e.g., `/home/<user>/workspace/.../text_to_speech/model` |
   | `englishModelPath` | Path to the XTTS v2 model directory | e.g., `/home/<user>/models/v2.0.2` |
   | `englishSpeakerWav` | Speaker reference WAV for XTTS v2 voice cloning (optional) | Absolute path to a `.wav` file; leave empty to use the Kinyarwanda conditioning audio |

   **Interface options explained:**

   | `interface` value | Effect |
   |-------------------|--------|
   | `service` | Exposes a ROS service at `/textToSpeech/say_text` (blocking call) |
   | `action` | Exposes a ROS action server at `/textToSpeech/say_text` (non-blocking with feedback) |
   | `both` | Exposes both simultaneously (useful during development) |

6. **Update Kinyarwanda model config.json paths:**

   Open the model configuration file:
   ```bash
   nano ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model/config.json
   ```
   
   Update the following lines to point to your `model/` directory:
   - Line 226: path to `speakers.pth`
   - Line 228: path to `speakers.pth`
   - Line 280: path to `speakers.pth`
   - Line 286: path to `speakers.pth`

   Replace `<user>` with your Linux username in each path.

7. **Make application files executable:**

   ```bash
   cd $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/src
   chmod +x text_to_speech_application.py
   ```

   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
     <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
     <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf" style="color: #66b3ff;">D5.5.2.4 Text-to-Speech Component</a>. Otherwise, the recommended values are the ones already set in the configuration file. Ensure the robot IP address matches your Pepper robot's network configuration.</span>
   </div>

8. **Run the `textToSpeech` node:**
   
   Follow the steps below, each in a separate terminal.

   - Source the workspace in the first terminal:
       ```bash
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
       ```
   - Launch the robot:
       ```bash
       roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=false launch_actuators:=true
       ```
       <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
        <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
        <span style="color: #cccccc;">Ensure that <code>robot_ip</code> and <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface.</span>
       </div>
   - Open a new terminal to launch the `textToSpeech` node:
       ```bash
         # Activate the virtual environment first
         source ~/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_text_to_speech_env/bin/activate
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash 
         rosrun cssr_system text_to_speech_application.py
       ```

   **Alternative: Launch robot and TTS node simultaneously:**
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
   roslaunch unit_tests text_to_speech_test_launch_robot.launch robot_ip:=<robot_ip> network_interface:=<network_interface>
   roslaunch unit_tests text_to_speech_test_launch_test_harness.launch run_tests:=false
   ```

## Simulator Robot
Does not have a simulator part.

## Executing Text-to-Speech Actions

Upon launching the node, the configured interface becomes available at `/textToSpeech/say_text`. Verify it is running:

```bash
# For service interface:
rosservice list | grep /textToSpeech

# For action interface:
rostopic list | grep /textToSpeech
```

---

### Using the ROS Service (`interface = service`)

The service uses the `cssr_system/TTS` message type:
- **Request**: `string message`, `string language`
- **Response**: `bool success`

```bash
rosservice call /textToSpeech/say_text "{message: '<text>', language: '<language>'}"
```

**English examples:**
```bash
rosservice call /textToSpeech/say_text "{message: 'Hello world', language: 'english'}"
rosservice call /textToSpeech/say_text "{message: 'How are you today?', language: 'english'}"
```

**Kinyarwanda examples:**
```bash
rosservice call /textToSpeech/say_text "{message: 'Muraho', language: 'kinyarwanda'}"
rosservice call /textToSpeech/say_text "{message: 'Amakuru?', language: 'kinyarwanda'}"
```

---

### Using the ROS Action Server (`interface = action`)

The action server uses the `cssr_system/TTSAction` message type:
- **Goal**: `string text`, `string language`
- **Feedback**: `string status`, `float32 progress` (0.0 → 1.0)
- **Result**: `bool success`, `string message`, `string audio_file_path`

The action server is non-blocking: it publishes feedback during synthesis and playback, and allows preemption (cancellation).

**Send a goal from the command line:**
```bash
# English
rostopic pub --once /textToSpeech/say_text/goal cssr_system/TTSActionGoal \
  "{goal: {text: 'Hello, I am Pepper.', language: 'english'}}"

# Kinyarwanda
rostopic pub --once /textToSpeech/say_text/goal cssr_system/TTSActionGoal \
  "{goal: {text: 'Muraho, nishimye kugira ikiganiro nawe.', language: 'kinyarwanda'}}"
```

**Monitor feedback and result:**
```bash
# In separate terminals:
rostopic echo /textToSpeech/say_text/feedback
rostopic echo /textToSpeech/say_text/result
```

**Send a goal from Python:**
```python
import rospy
import actionlib
from cssr_system.msg import TTSAction, TTSGoal

rospy.init_node('tts_client')
client = actionlib.SimpleActionClient('/textToSpeech/say_text', TTSAction)
client.wait_for_server()

goal = TTSGoal(text='Hello, I am Pepper.', language='english')
client.send_goal(goal)
client.wait_for_result()

result = client.get_result()
print(result.success, result.message)
```

---

### Language and Interface Reference

| Parameter | Value | Description |
|-----------|-------|-------------|
| `language` | `english` | Synthesized with XTTS v2 (22050 Hz output) |
| `language` | `kinyarwanda` | Synthesized with fine-tuned YourTTS (16000 Hz output) |
| `interface` | `service` | Blocking ROS service — waits until audio finishes playing |
| `interface` | `action` | Non-blocking ROS action — publishes progress feedback |
| `interface` | `both` | Both interfaces active simultaneously |
| `playback_mode` | `naoqi` | Transfer WAV to robot via SSH, play with ALAudioPlayer |
| `playback_mode` | `local` | Play on local machine via `paplay` |

---

## Troubleshooting

**Common Issues:**

1. **Service not available:**
   ```
   ERROR: Service [/textToSpeech/say_text] is not available.
   ```
   **Solution**: Ensure the TTS node is running and the virtual environment is activated. Check that `interface` is set to `service` or `both` in the configuration file.

2. **Action server not available:**
   ```
   [ERROR] Failed to connect to action server /textToSpeech/say_text
   ```
   **Solution**: Ensure `interface` is set to `action` or `both` in the configuration file and restart the node.

3. **Kinyarwanda model file not found:**
   ```
   Unable to load synthesizer: [Errno 2] No such file or directory: '.../model/config.json'
   ```
   **Solution**: Verify all Kinyarwanda model files are in the `model/` directory and that `kinyarwandaModelPath` in the configuration file points to the correct location.

4. **English XTTS v2 model not found:**
   ```
   [Errno 2] No such file or directory: '.../models/v2.0.2/config.json'
   ```
   **Solution**: Download the XTTS v2 model (see step 4) and verify `englishModelPath` in the configuration file.

5. **Robot connection / playback failure:**
   ```
   Error in say_text: Command '['/usr/bin/python2', 'send_and_play_audio.py', ...]' returned non-zero exit status 1.
   ```
   **Solution**: Check the robot IP address and network connectivity, ensure the robot is powered on, and verify `sshpass` is installed. To test synthesis without the robot, set `playback_mode = local`.

6. **Virtual environment issues:**
   ```
   ModuleNotFoundError: No module named 'TTS'
   ```
   **Solution**: Activate the virtual environment before running the node.

7. **Python version conflicts:**
   ```
   TypeError: unsupported operand type(s)
   ```
   **Solution**: Confirm you are using Python 3.10 with `python --version` inside the virtual environment.

8. **CUDA warnings:**
   ```
   CUDA requested but not available. Falling back to CPU.
   ```
   **Solution**: Either install CUDA/PyTorch with GPU support or set `useCuda = False` in the configuration file.

## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the TTS system architecture, neural model details, configuration options, multilingual capabilities, and advanced troubleshooting procedures, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf" style="color: #66b3ff;">D5.5.2.4 Text-to-Speech report</a>. This manual provides comprehensive technical documentation essential for development, customization, and maintenance of the TTS system.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:muhirwarichard1@gmail.com">muhirwarichard1@gmail.com</a>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

**Authors:** Muhirwa Richard, CSSR4Africa Consortium  
**Date:** May 2025
