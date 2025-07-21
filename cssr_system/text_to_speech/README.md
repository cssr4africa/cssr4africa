
<div align="center">
  <h1>Integrated Text-to-Speech</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `textToSpeech` ROS node provides multilingual text-to-speech functionality for the Pepper robot within the CSSR4Africa project. This component enables the robot to speak in multiple languages including English and Kinyarwanda, supporting both built-in robot TTS capabilities and custom neural TTS models. The node exposes a ROS service interface that accepts text messages and language specifications, then generates and plays the corresponding speech audio through the robot's audio system.

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

   Verify the model files are in the `model/` directory:

   ```bash
   # Verify the models are in the models directory:
   ls ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model

   # If there is no output, use the commands below to obtain the models:

   # If git-lfs is already set up, skip this section
   sudo apt-get update && sudo apt-get install git-lfs
   cd && git lfs install

   # Clone the models from HuggingFace:
   git clone https://huggingface.co/cssr4africa/cssr4africa_models

   # Copy the text-to-speech model to the 'model/' directory:
   mkdir -p ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model
   unzip cssr4africa_models/text_to_speech/model.zip -d ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech
   ```

4. **Update Configuration File:**
   
   Navigate to the configuration file located at `~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/config/text_to_speech_configuration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `language` | Default language for TTS | `english` or `kinyarwanda` |
   | `verboseMode` | Enable detailed logging | `True` or `False` |
   | `ip` | Pepper robot IP address | e.g., `172.29.111.240` |
   | `port` | Robot communication port | `9559` (default) |
   | `useCuda` | Enable GPU acceleration | `True` or `False` |

5. **Make application files executable:**

   ```bash
   cd $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/src
   chmod +x text_to_speech_application.py
   
   ```

   **Update Model Paths:**
   Open the model configuration file:
   ```bash
   nano ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model/config.json
   ```
   
   Change the following paths:
   - Line 226: Change to `/home/<user>/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/model/speakers.pth`, where `<user>` is your username
   - Line 228: Update with the same path
   - Line 280: Update with the same path
   - Line 286: Update with the same path

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color:   #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf" style="color: #66b3ff;">D5.5.2.4 Text-to-Speech Component</a>. Otherwise, the recommended values are the ones already set in the configuration file. Ensure the robot IP address matches your Pepper robot's network configuration.</span>
  </div>

6. **Run the `textToSpeech` from `cssr_system` package:**
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
        roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=false launch_actuators:=true
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP address <code>robot_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
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
Does not have a simulator part

## Executing Text-to-Speech Actions
Upon launching the node, the hosted service (`/textToSpeech/say_text`) is available and ready to be invoked. This can be verified by running the following command in a new terminal:

```bash
rosservice list | grep /textToSpeech
```

The command below invokes the service to execute text-to-speech actions (run in a new terminal) with the request parameters defined below:

```bash
rosservice call /textToSpeech/say_text -- message language
```

### Service Request Parameters
#### 1. Message (message)
- **Type**: `string`
- **Description**: The text content to be synthesized and spoken by the robot
- **Examples**: 
  - `"Hello world"` - Simple English greeting
  - `"How are you today?"` - English question
  - `"Muraho"` - Kinyarwanda greeting
  - `"Amakuru?"` - Kinyarwanda question

#### 2. Language (language)
- **`english`**: Uses the robot's built-in English TTS system for speech synthesis
- **`kinyarwanda`**: Uses the custom neural TTS model for Kinyarwanda speech synthesis

### Sample Invocations

**English Examples:**
```bash
# Method 1: Inline format with braces
rosservice call /textToSpeech/say_text "{message: 'Hello world', language: 'english'}"

# Method 2: Simple positional arguments
rosservice call /textToSpeech/say_text "Hello, how are you?" "english"
```

**Kinyarwanda Examples:**
```bash
# Question in Kinyarwanda
rosservice call /textToSpeech/say_text "{message: 'Amakuru?', language: 'kinyarwanda'}"

# Longer sentence in Kinyarwanda
rosservice call /textToSpeech/say_text "Muraho, ubu ni ubutumwa bwo kugenzura." "kinyarwanda"
```

**Testing Different Languages:**
```bash
# Test language switching
rosservice call /textToSpeech/say_text "This is English" "english"
rosservice call /textToSpeech/say_text "Iyi ni Ikinyarwanda" "kinyarwanda"
rosservice call /textToSpeech/say_text "Back to English" "english"
```

## Troubleshooting

**Common Issues:**

1. **Service not available:**
   ```
   ERROR: Service [/textToSpeech/say_text] is not available.
   ```
   **Solution**: Ensure the TTS node is running and the virtual environment is activated.

2. **Model file not found:**
   ```
   Unable to load synthesizer: [Errno 2] No such file or directory
   ```
   **Solution**: Verify all model files are in the correct location and paths in config.json are updated.

3. **Robot connection issues:**
   ```
   Failed to send message to robot
   ```
   **Solution**: Check robot IP address, network connectivity, and ensure robot is powered on.

4. **Virtual environment issues:**
   ```
   ModuleNotFoundError: No module named 'TTS'
   ```
   **Solution**: Activate the virtual environment before running the TTS node.

5. **Python version conflicts:**
   ```
   TypeError: unsupported operand type(s)
   ```
   **Solution**: Confirm you're using Python 3.10 with `python --version` in the virtual environment.

6. **CUDA warnings:**
   ```
   CUDA requested but not available. Falling back to CPU.
   ```
   **Solution**: Either install CUDA/PyTorch or set `useCuda = False` in configuration.

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