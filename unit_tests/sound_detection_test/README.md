<div align="center">
  <h1>Sound Detection and Localization Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the **Sound Detection and Localization**, a node within the CSSR4Africa project (**cssr_system** package). The node evaluates the performance of sound detection and direction localization under different conditions. It records both filtered and unfiltered audio to analyze the effectiveness of the audio processing algorithms, and generates visual plots comparing raw and processed signals. The tests assess the accuracy of sound source direction detection, ensuring the system can correctly localize sounds on the horizontal plane. Additionally, the module verifies that the voice activity detection (VAD) correctly identifies when speech is present in the audio stream, enabling reliable human-robot auditory interaction.

---

#  📄 Documentation
The main documentation for this deliverable is found in **[D4.2.3 Sound Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.3.pdf)**, which provides more details regarding how to run the test and gives detail about the sound detection and localization module.

---

#  Configuration File
The following table provides the key-value pairs used in the configuration file:

## 🔧 Configuration Parameters

The following parameters are used to configure and run the **Sound Detection Test Node**. These settings control how audio is recorded, processed, and logged during operation.

| Parameter               | Description                                      | Possible Values            | Default Value |
|-------------------------|--------------------------------------------------|----------------------------|---------------|
| `recordFiltered`        | Record filtered audio output                     | `true`, `false`            | `true`        |
| `recordUnfiltered`      | Record unfiltered (raw) audio input              | `true`, `false`            | `true`        |
| `recordDuration`        | Duration (seconds) for recorded audio            | Positive integer           | `10`          |
| `saveDirectionData`     | Save direction data (e.g., for localization)     | `true`, `false`            | `true`        |
| `targetRMS`             | Target RMS level for audio normalization         | Positive float             | `0.2`         |
| `applyNormalization`    | Apply RMS-based normalization to audio           | `true`, `false`            | `true`        |
| `verboseMode`           | Enable detailed logging and diagnostics          | `true`, `false`            | `true`        |

---

> **Note:**  
> Enabling **`verboseMode`** (`true`) will activate detailed logging and provide diagnostic information about buffer sizes, publishing rates, and other metrics useful for debugging.

#  🚀 Run the Sound Detection Unit Test
Before running the node, the configuration file must be set up correctly with the appropriate key-value pairs. The configuration file is typically located in the package folder under the `config` directory.

The test can be performed on a **physical robot using its microphones** or using **pre-recorded audio saved as a ROS bag file**. The specific test to run can be selected by modifying the configuration.

1. **Install the required software components:**

   Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf)

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

3. **Update Configuration File**
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/sound_detection_test/config/sound_detection_test_configuration.json` and update the configuration according to the key-value pairs as shown above.


## Prerequisites
Before running the Test Harness, activate the sound detection Python environment:
```bash
source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_sound_detection_env/bin/activate
```

Make the application scripts executable:
```bash
chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/src/sound_detection_application.py
chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/sound_detection_test/src/sound_detection_test_application.py
```

## Usage

### Launch the Physical Robot
```bash
# Launch the robot with the appropriate IP and network interface
roslaunch unit_tests sound_detection_test_launch_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> use_recorded_audio:=false
```

### Using Pre-recorded Audio
```bash
# Use a pre-recorded audio file
roslaunch unit_tests sound_detection_test_launch_robot.launch use_recorded_audio:=true audio_file:=<audio_file>
```

#### Audio File Options (`audio_file`):
- **`sound_distance`**: Standard speech at normal volume and distance
- **`sound_angle`**: Multiple speakers from different directions
- **`sound_noise`**: Speech with background noise for robustness testing

### Run Sound Detection Test Harness
```bash
# The launch file launches the sound_detection node for the unit test
roslaunch unit_tests sound_detection_test_launch_test_harness.launch
```

## Available RosBag Files for Testing

The test suite includes specialized rosbag files for different testing scenarios:

### 1. `sound_distance.bag`
- **Purpose**: Testing sound filtering effectiveness at various distances
- **Description**: Contains recordings of speech at different distances from the robot
- **Expected Results**: Successful detection and filtering of speech across different distances

### 2. `sound_angle.bag`
- **Purpose**: Testing angle detection accuracy
- **Description**: Contains recordings of sound sources moving from the right side of the robot (positive angle) to the left side (negative angle)
- **Expected Results**: Accurate tracking of sound source angle throughout movement

### 3. `sound_noise.bag`
- **Purpose**: Testing noise filtering and speech detection
- **Description**: Contains recordings with intentional pauses between speech (e.g., counting numbers with pauses)
- **Expected Results**: The system should detect only speech segments and filter out non-speech portions, resulting in continuous number counting in the processed output

# 🖥️ Output Result

When running the sound detection test node, the following outputs are generated:

### Audio Recordings
- **Noise reduced audio files**: `sound_detection_test_noise_filtered_audio_YYYY-MM-DD_HH-MM-SS.wav`
  - Processed audio with only noise reduction and filtering.

- **Filtered audio files**: `sound_detection_test_speech_filtered_YYYY-MM-DD_HH-MM-SS.wav`
  - Processed audio with applied noise reduction and filtering
  - Speech detection is applied to separate non-speech part
  - Optionally RMS normalized for consistent volume levels

- **Unfiltered raw audio files**: `sound_detection_test_unfiltered_YYYY-MM-DD_HH-MM-SS.wav`
  - Raw audio captured directly from the microphone
  - Optionally RMS normalized for consistent volume levels

### Direction Data
- **Direction text files**: `sound_detection_test_direction_data_YYYY-MM-DD_HH-MM-SS.txt`
  - Contains timestamps and corresponding direction angles
  - Format: `timestamp_in_seconds, angle_in_degrees`
  - Negative angles indicate sound from left side, positive angles from right side

For detailed information about the test results and system performance, refer to the Sound Detection and Localization Deliverable Report.

# 💡Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

# 📜 License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by the African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme 

2025-04-13