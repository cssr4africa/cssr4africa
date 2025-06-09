<div align="center">
  <h1>Speech Event Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `speechEvent` ROS node within the CSSR4Africa project (`cssr_system` package).
Details about the main `speechEvent` ROS node are located in the `~/workspace/pepper_rob_ws/src/cssr_system/speech_event/README.md`
file. The results are logged in the file `~/workspace/pepper_rob_ws/src/unit_tests/speech_event_test/data/speech_event_test_output.dat`.

These tests check to ensure that `speechEvent` works as expected end-to-end, from when it receives a signal on the
`\soundDetection\signal` ROS topic to when it publishes transcribed text on the `\speechEvent\text` ROS topic. They also
check that the `\speechEvent\set_language` ROS service that sets the `speechEvent` transcription language at runtime
works as expected.

The test report located in the file `~/workspace/pepper_rob_ws/src/unit_tests/speech_event_test/data/speech_event_test_output.dat`
borrows its style from [Behaviour Driven Development](https://en.wikipedia.org/wiki/Behavior-driven_development). Even though `speechEvent` was
not developed using Behaviour Driven Development, the test report adopts its manner of writing requirements in a testable
way so that each single test is easily linked to a specific requirement that it is testing.

# Documentation

Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests.
The deliverable report can be found in [D4.3.2 Speech Event](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.3.2.pdf).

# Run the Speech Event Unit Test

## Physical Robot

### Steps

1. **Install the required software components:**

    Install the required software components to instantiate and set up the development environment for controlling the Pepper
    robot in both physical and simulated environments. Use the
    [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

2. **Clone and build the project (if not already cloned)**:

   - Move to the source directory of the workspace

      ```bash
         cd $HOME/workspace/pepper_rob_ws/src
       ```

   - Clone the `CSSR4Africa` software from the GitHub repository

      ```bash
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```

   - Install required linux tools

      ```bash
      sudo apt-get update
      
      sudo apt-get install cython3 ffmpeg gfortran libopenblas-dev libopenblas64-dev patchelf pkg-config python3-testresources python3-tk python3-typing-extensions sox
       ```

   - Create a Python virtual environment and install required Python packages (Speech Event has been tested and proven
   to work using Python3.8)

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
   `~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/speech_event/config/speech_event_configuration.ini` and update
   the configuration according to the key-value pairs below (this step configures `speechEvent` ROS node):

   | Key                | Value                    | Description                                                                                                                 |
   | ------------------ | ------------------------ | --------------------------------------------------------------------------------------------------------------------------- |
   | language           | kinyarwanda \| english   | Specifies the language in which the utterance is spoken.                                                                    |
   | verboseMode        | true \| false            | Specifies whether diagnostic data is to be printed to the terminal.                                                         |
   | cuda               | true \| false            | Specifies whether to use GPUs. The term ‘cuda’ is chosen as the key to alert the user that only NVIDIA GPUs are supported.  |
   | confidence         | \<float>                 | The confidence level on a scale of 0 to 1 above which transcriptions are are assumed to be acceptable and correct.          |
   | speechPausePeriod  | \<float>                 | The time period above which one utterance is assumed to be separate from a preceding utterance.                             |
   | maxUtteranceLength | \<int>                   | The maximum length (in seconds) of an utterance. Longer utterances are split when they go past this length.                 |
   | sampleRate         | \<int>                   | Specifies the sampling rate of the incoming audio sourced from the /soundDetection/signal ROS topic.                        |
   | heartbeatMsgPeriod | \<int>                   | Specifies the time period in seconds at which a periodic heartbeat message is sent to the terminal.                               |

   Navigate to the configuration file located at
   `~/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/speech_event_test/config/speech_event_test_configuration.ini` and
   update the configuration according to the key-value pairs below (this step configures `speechEventTest` ROS node):

   | Key                | Value       | Description                                                                                                             |
   | ------------------ | ----------- | ----------------------------------------------------------------------------------------------------------------------- |
   | heartbeatMsgPeriod | \<int>      | Specifies the time period in seconds at which a periodic heartbeat message is sent to the terminal.                     |
   | waitTimeout        | \<int>      | Specifies how long to wait for speechEvent to start.                                                                    |
   | mode               | mic \| file | Use 'mic' when using real-time utterances from Pepper or the driver ROS node, and 'file' when using saved audio files.  |

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

   NOTE: Tweak the speechAmplitudeThreshold in case the driver is having trouble capturing speech utterances. Increase
   it in case speech is not being detected, and decrease it in case ambient noise is being captured together with the
   speech utterances. You may also tweak the sensitivity of the PC's microphones in case speechAmplitudeThreshold fails
   to solve the aforementioned issues that may arise.

   When using the driver to test speechEvent, ensure that `mode` in the driver configuration file matches mode in
   speechEventTest configuration file (either set both to `file` or set both to `mic`). When using a running soundDetection
   ROS node to test speechEvent, ensure that `mode` in the speechEventTest configuration is set to `mic`.

   If using ROS launch files to launch the test harness, the argument passed to the `mode` parameter of the launch file
   will override whatever is set in the configuration files. Therefore when using the test harness launch file, no need
   to update the `mode` option in the configuration files, as setting it once while launching using the test harness
   launch file suffices.

4. **Run the `speechEventTest` from the`unit_tests`  package**

    Follow below steps, run in different terminals.

    - Source the workspace in first terminal:

        ```bash
        cd $HOME/workspace/pepper_rob_ws

        source devel/setup.bash
        ```

    - Launch the robot (skip this step if testing using a driver):
  
        ```bash
        # Launch the robot with the appropriate IP and network interface
        roslaunch unit_tests speech_event_test_launch_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip>  network_interface:=<network_interface> 
        ```

        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>

    - Open a new terminal to launch the `speechEventTest` (which launches the speechEvent ROS node and run tests on it).
    This creates a driver for the `/soundDetection/signal` ROS topic.

        ```bash
        source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/bin/activate

        cd $HOME/workspace/pepper_rob_ws

        source devel/setup.bash

        roslaunch unit_tests speech_event_test_launch_test_harness.launch launch_driver:=true run_tests:=true mode:=file
        ```

        Update the `launch_driver` to either `true` or `false` depending on whether you are using a driver or a running
        soundDetection ROS launch. To skip running tests, set the `run_tests` parameter to `false`. If using the
        soundDetection ROS node or PC microphones set `mode` to `mic`, and if using pre-recorded saved files set `mode`
        to `file`.

        If sourcing the Python virtual environment fails to work, replace it with:
        `export PYTHONPATH=$HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_speech_event_env/lib/python3.8/site-packages:$PYTHONPATH`

## Simulator Robot

  This ROS node does not support the simulator as it depends on Sound Detection that does not also support the simulator,
  as the simulator provides no audio input capability that Sound Detection requires

## Tests Executed

### Test `/speechEvent/set_language`

This test sets the `speechEvent` ROS node at runtime to the following languages:

- `Kiswahili`
- `Arabic`
- `Kinyarwanda`
- `English`

Speech Event supports only Kinyarwanda and English. Therefore, invoking `/speechEvent/set_language` with Kiswahili or
Arabic should not update the transcription language but instead the service whould return 0 to show that the language
has not been updated. On the other hand, invoking the service with Kinyarwanda or English should update the transcription
language, and the service should return a response of 1 to show that the language has been updated.

### Test the end-to-end speechEvent transcription process

This test either uses saved audio files or real-time utterances from either Pepper's or the testing PC's microphones.

When using saved files, a series of wav files whose utterances are known before-hand are transcribed and the results
compared with the ground truth values to check whether the test has passed. In this mode, both the driver and the
`speechEventTest` ROS nodes have to have their mode set to `file`. The following are the utterances contained in the
audio files tested in this mode:

- `amakuru`
- `murakaza neza`

When using real-time utterances spoken by the tester either using Pepper or the Speech Event driver ROS node, a series of
utterances are collected over a period of 60 seconds and their text transcriptions saved to the test report
`~/workspace/pepper_rob_ws/src/unit_tests/speech_event_test/data/speech_event_test_output.dat`. It's advisable to have
a pre-determined list of utterances to speak out loud, and once the test is completed, compare the transcriptions in the
test report to check if they are correct. In this mode, both the driver and the `speechEventTest` ROS nodes have to have
their mode set to `mic`. If using Pepper, then the driver ROS node can be ignored.

## Results

The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/speech_event_test/data/speech_event_test_output.dat`
file. As previously stated in the introduction of this document, the test report's style borrows from Behaviour Driven
Development. Below is a sample of how the test report should appear after running tests:

```text
Speech Event Test Report
    
Date: 2025-04-20 18:34:47
    
    
================================================================================
    
Test SpeechEvent's /speechEvent/set_enabled ROS service
---
As a:     behaviourController developer
I want:   a means of enabling and disabling the transcription process at runtime
So that:  I can stop the robot from transcribing its own speech by disabling the transcription process
---
Set unsupported status (agree): PASS
Set unsupported status (1): PASS
Set supported status (false): PASS
Set supported status (true): PASS

================================================================================

Test SpeechEvent's /speechEvent/set_language ROS service
---
As a:     behaviourController developer
I want:   a means of setting the transcription language of speechEvent at runtime
So that:  I don't have to restart speechEvent every time I update the transcription language
---
Set unsupported language (Kiswahili): PASS
Set unsupported language (Arabic): PASS
Set supported language (English): PASS
Set supported language (Kinyarwanda): PASS

================================================================================

Test SpeechEvent's end-to-end transcription process
---
Given:    a running speechEvent ROS node
When:     an audio signal is detected on the /soundDetection/signal ROS topic
Then:     the audio needs to be transcribed and the text transcription published on the /speechEvent/text ROS topic
---

Transcribe 'rw-ingendo': PASS
Transcribe 'rw-atandatu': PASS
Transcribe 'rw-ibikenewe': PASS
Transcribe 'rw-kabiri': PASS
Transcribe 'en-turner construction was the construction manager for the project': PASS
Transcribe 'en-he now resides in monte carlo': PASS
Transcribe 'en-the council was held': PASS
```

<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the exampleComponentTest node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.3.2.pdf" style="color: #66b3ff;">D4.3.2 Speech Event</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:

- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:email@andrew.cmu.edu">email@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  

Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-04-20
