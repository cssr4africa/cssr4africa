<div align="center">
  <h1>Speech Event</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `speechEvent` ROS node performs automatic speech recognition (ASR), transcribing both Kinyarwanda and English
utterances. The ASR models used are acquired from the following sources:
[Kinyarwanda model](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/nemo/models/stt_rw_conformer_transducer_large),
[English model](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/nemo/models/stt_en_conformer_transducer_large). These
ASR models need to be downloaded and stored in the `./speech_event/models/` directory.

# Documentation

Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the
tests. The deliverable report can be found in
[D4.3.2 Speech Event](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.3.2.pdf).

# Run the Example Component Node

## Physical Robot

### Steps

1. **Install the required software components:**

   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use
   the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

2. **Clone and build the project (if not already cloned)**:

   - Move to the source directory of the workspace

      ```bash
      cd $HOME/workspace/pepper_rob_ws/src
       ```

   - Clone the `CSSR4Africa` software from the GitHub repository

      ```bash
      git clone https://github.com/cssr4africa/cssr4africa.git
       ```

   - Verify the model files are in the models directory. 
   
      ```sh
      # Verify the models are in the models directory:
      ls ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/models

      # If there is no output, use the commands below to obtain the models:

      # Clone the models from HuggingFace:
      cd && git lfs install

      git clone https://huggingface.co/cssr4africa/cssr4africa_models

      # Move the models to the models directory:
      mv cssr4africa_models/speech_event/models/* ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/models
      ```

   - Install required linux tools

      ```bash
      sudo apt-get update

      sudo apt-get install cython3 ffmpeg gfortran libopenblas-dev libopenblas64-dev patchelf pkg-config portaudio19-dev python3-testresources python3-tk python3-typing-extensions sox
       ```

   - Create a Python virtual environment and install required Python packages (Speech Event has been tested and proven to work using Python3.8)

      ```bash
      mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs

      cd $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs

      python3 -m venv cssr4africa_speech_event_env

      source cssr4africa_speech_event_env/bin/activate

      pip install --upgrade pip

      pip install -r $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/speech_event_requirements.txt
       ```

   - Build the ROS workspace (it's best to proceed with the next steps on a separate terminal, otherwise an error may arise when running `catkin_make`)

      ```bash
      cd $HOME/workspace/pepper_rob_ws

      catkin_make

      source devel/setup.bash
       ```

   - Make application files executable

      ```bash
      chmod +x $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/src/speech_event_application.py

      chmod +x $HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/speech_event_test/src/speech_event_test_application.py

      chmod +x $HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/speech_event_test/src/speech_event_driver.py
       ```

3. **Update Configuration File:**

   Navigate to the configuration file located at
   `~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/config/speech_event_configuration.ini` and update the
   configuration according to the key-value pairs below:

   | Key                | Value                    | Description                                                                                                                 |
   | ------------------ | ------------------------ | --------------------------------------------------------------------------------------------------------------------------- |
   | language           | kinyarwanda \| english   | Specifies the language in which the utterance is spoken.                                                                    |
   | verboseMode        | true \| false            | Specifies whether diagnostic data is to be printed to the terminal.                                                         |
   | cuda               | true \| false            | Specifies whether to use GPUs. The term ‘cuda’ is chosen as the key to alert the user that only NVIDIA GPUs are supported.  |
   | confidence         | \<float>                 | The confidence level on a scale of 0 to 1 above which transcriptions are are assumed to be acceptable and correct.          |
   | speechPausePeriod  | \<float>                 | The time period above which one utterance is assumed to be separate from a preceding utterance.                             |
   | maxUtteranceLength | \<int>                   | The maximum length (in seconds) of an utterance. Longer utterances are split when they go past this length.                 |
   | sampleRate         | \<int>                   | Specifies the sampling rate of the incoming audio sourced from the /soundDetection/signal ROS topic.                        |
   | heartbeatMsgPeriod | \<int>                   | Specifies the time period in seconds at which a periodic heartbeat message is sent to the terminal.                            |

   Navigate to the configuration file located at
   `~/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/speech_event_test/config/speech_event_test_driver.ini` and update
   the configuration according to the key-value pairs below  (this step configures the Speech Event driver ROS node):

   | Key                      | Value       | Description                                                                                                   |
   | ------------------------ | ----------- | ------------------------------------------------------------------------------------------------------------- |
   | channels                 | \<int>      | Number of audio channels to use when acquiring audio from PC's microphones.                                   |
   | chunkSize                | \<int>      | Number of samples to acquire at a go every time that audio is read from the PC's microphones.                 |
   | sampleRate               | \<int>      | The sample rate to use when acquiring audio from the PC's microphones.                                        |
   | speechAmplitudeThreshold | \<float>    | Threshold above which a signal sample is assumed to contain a speech utterance.                               |
   | mode                     | mic \| file | Use 'mic' when using real-time utterances from the PC's microphones, and 'file' when using saved audio files. |

4. **Run the `speechEvent` ROS node from the `cssr_system`  package:**

   Follow below steps, run in different terminals.

    - Source the workspace in first terminal:

        ```bash
        cd $HOME/workspace/pepper_rob_ws
        
        source devel/setup.bash
        ```

    - Launch the robot (skip this step if testing using a driver):
  
        ```bash    
        roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=true launch_actuators:=false 
        ```

        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. 
      </div>

    - Open a new terminal to launch the `speechEvent` ROS node.

        ```bash
        source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/bin/activate

        cd $HOME/workspace/pepper_rob_ws

        source devel/setup.bash

        rosrun cssr_system speech_event_application.py
        ```

        If sourcing the Python virtual environment fails to work, replace it with:
        `export PYTHONPATH=$HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/lib/python3.8/site-packages:$PYTHONPATH`

      N.B: Running the `speechEvent` ROS node requires the `/soundDetection/signal` topic to be available, which can be
      hosted by running the `soundDetection` ROS node in the `cssr_system` package or running the `speech_event_driver`
      in the `unit_tests` package before running the `speechEvent` ROS node in the step above:

         - (Option 1): Run the `soundDetection` ROS node of the `cssr_system` package (in a new terminal):

         ```sh
        source ~/workspace/pepper_rob_ws/devel/setup.bash

        # Activate the python environment
        source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_sound_detection_env/bin/activate

        rosrun cssr_system sound_detection_application.py
         ```

         - (Option 2): Run the `speech_event_driver` of the `unit_tests` package (in a new terminal, and after updating
         the mode to 'mic' in the `speech_event_driver.ini` configuration file in the
         `$HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/speech_event_tests/config` directory):

         ```sh
         source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/bin/activate

         source ~/workspace/pepper_rob_ws/devel/setup.bash

         rosrun unit_tests speech_event_driver.py
         ```

         The details about the 'mode' key in the speechEvent driver configuration are found in the `Update Configuration File`
         section of this README.md file.

         If sourcing the Python virtual environment fails to work, replace it with:

         ```sh
         export PYTHONPATH=$HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/lib/python3.8/site-packages:$PYTHONPATH
         ```

    - SpeechEvent can also be run using a ROS launch file:

        ```bash
          source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/bin/activate

          cd $HOME/workspace/pepper_rob_ws

          source devel/setup.bash

          roslaunch unit_tests speech_event_test_launch_test_harness.launch launch_driver:=true run_tests:=false mode:=mic
        ```

        Update the `launch_driver` to either `true` or `false` depending on whether you are using a driver or a running
        soundDetection ROS launch. To skip running tests, set the `run_tests` parameter to `false`. If using the
        soundDetection ROS node or PC microphones set `mode` to `mic`, and if using pre-recorded saved files set `mode`
        to `file`.

## Simulator Robot

This ROS node does not support the simulator as it depends on Sound Detection that does not also support the simulator,
as the simulator provides no audio input capability that Sound Detection requires

## Setting the Transcription Language at Runtime

Upon launching the node, the hosted service (`/speechEvent/set_language`) is available and ready to be invoked. This can
be verified by running the following command in a new terminal:

```sh
rosservice list | grep /speechEvent
```

The command below invokes the service to update the transcription language at runtime (the placeholder `<language>` is to
be replaced with either 'Kinyarwanda' or 'English'):

```sh
rosservice call /speechEvent/set_language <language>
```

A response of 1 indicates that the service invocation was successful, and the language has been updated successfully. A
response of 0, on the other hand, indicates an error such as when using an unsupported language such as 'Arabic'.

### Sample Invocations

- `rosservice call /speechEvent/set_language Kinyarwanda`
- `rosservice call /speechEvent/set_language English`

## Enabling and Disabling the Transcription Process

Upon launching the node, the hosted service (`/speechEvent/set_enabled`) is available and ready to be invoked. This can
be verified by running the following command in a new terminal:

```sh
rosservice list | grep /speechEvent
```

The command below invokes the service to enable or disable the transcription process at runtime (the placeholder
`<status>` is to be replaced with either 'true' or 'false'):

```sh
rosservice call /speechEvent/set_enabled "{status: '<status>'}"
```

A response of 1 indicates that the service invocation was successful, and the status has been updated successfully. A
response of 0, on the other hand, indicates an error such as when using an unsupported status such as 'agree' or '1'.

### Sample Invocations

- `rosservice call /speechEvent/set_enabled "{status: 'true'}"`
- `rosservice call /speechEvent/set_enabled "{status: 'false'}"`

## Acquiring Text Transcriptions

Upon launching the node, the hosted topic (`/speechEvent/text`) is available and ready to be used. This can be verified
by running the following command in a new terminal:

```sh
rostopic list | grep /speechEvent
```

To view the transcriptions on the terminal:

```sh
rostopic echo /speechEvent/text
```

When the `verboseMode` configuration option is set to `true` in `speech_event_configuration.ini` located in
`$HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/config`, transcriptions will also be viewable on
a GUI application.

<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the speechEvent ROS node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.3.2.pdf" style="color: #66b3ff;">D4.3.2 Speech Event</a>.These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>

## Support

For issues or questions:

- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:cliffor2@andrew.cmu.edu">cliffor2@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  

Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-04-20
