<div align="center">
<h1> Sound Detection and Localization </h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Sound Detection and Localization** package is a ROS package designed to detect conspicuous sounds, specifically human voices, and determine their direction of arrival in real-time by processing audio signals from the robot's microphones. The package calculates the azimuth angle of arrival, representing the sound's direction on the horizontal plane relative to the robot's Cartesian head frame. It publishes this angle to the **/soundDetection/direction** topic, allowing the robot to direct its gaze toward the detected sound source. Additionally, the captured audio signal is published to the **/soundDetection/signal** topic, from onset to offset of the detected sound. In verbose mode, the module provides diagnostic output to the terminal for debugging. This package enables the robot to localize and respond to human voices, filtering out ambient noise and reverberation to maintain precise and responsive auditory localization in real-time.

# 📄 Documentation
The main documentation for this deliverable is found in [D4.2.3 Sound Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.3.pdf) that provides more details.

# 🛠️ Installation 

Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

## Installation on Ubuntu (x86-based Systems)

1. Prerequisites  
Make sure you are running Ubuntu 20.04. You'll need to set up a Python virtual environment for the sound detection module.

2. Install Python Virtual Environment
```sh
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install Python virtual environment tools
sudo apt install python3.8-venv -y

# Create a virtual environment
cd $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/
python3.8 -m venv cssr4africa_sound_detection_env

# Activate the virtual environment
source cssr4africa_sound_detection_env/bin/activate

# Upgrade pip in the virtual environment
pip install --upgrade pip

# Install required packages
pip install -r ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/sound_detection_requirements.txt
```

# 🔧 Configuration Parameters  
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                     | Range/Values               | Default Value |
|-----------------------------|-----------------------------------------------------------------|----------------------------|---------------|
| `intensityThreshold`        | Minimum intensity threshold for audio processing                | Positive float             | `0.0039`      |
| `distanceBetweenEars`       | Distance between microphones in meters                          | Positive float             | `0.07`        |
| `localizationBufferSize`    | Size of audio buffer for localization                           | Positive integer           | `16384`        |
| `vadAggressiveness`         | Voice Activity Detection aggressiveness level                   | `[0-3]`                    | `1`           |
| `contextDuration`           | Duration (in seconds) of context window for processing          | Positive float (seconds)   | `1.0`         |
| `useNoiseReduction`         | Enable noise reduction                                          | `true`, `false`            | `true`        |
| `stationary`                | Assume noise is stationary during noise reduction               | `true`, `false`            | `true`        |
| `propDecrease`              | Proportion of noise reduction applied to signal                 | `[0.0 - 1.0]`              | `0.9`         |
| `audioTimeout`              | Timeout (seconds) for shutting down the node after audio ends   | Positive float (seconds)   | `2.0`         |
| `verboseMode`               | Enable detailed logging and diagnostic information              | `true`, `false`            | `true`        |

> **Note:**  
> Enabling **`verboseMode`** (`true`) will provide detailed diagnostic output to the terminal.

# 🚀 Running the node
**Run the `soundDetection` from the `cssr_system` package:**

Source the workspace in the first terminal:
```bash
cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
```

Follow below steps, run in different terminals.

1️⃣ Launch the robot:
```bash
  roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=true launch_actuators:=false
  ```
  <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. 
  </div>

2️⃣ Run the Sound Detection and Localization:

In a new terminal, activate the Python environment:
```bash
# Activate the python environment
source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_sound_detection_env/bin/activate
```

```bash
# Command to make application executable
chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/src/sound_detection_application.py
```

```bash
# Run the sound_detection node
rosrun cssr_system sound_detection_application.py
```

# 🖥️ Output
The node publishes two types of data:

1. **Processed Audio Signal**  
   Topic: `/soundDetection/signal`  
   Type: `std_msgs/Float32MultiArray`  
   Description: Contains the processed audio signal with noise reduction and filtering applied.

2. **Sound Direction Angle**  
   Topic: `/soundDetection/direction`  
   Type: `std_msgs/Float32`  
   Description: Contains the azimuth angle (in degrees) of the detected sound source relative to the robot's head frame.

You can verify the publication status using the following commands:
```bash
rostopic echo /soundDetection/signal
rostopic echo /soundDetection/direction
```
# 💡 Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

# 📜License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

2025-04-27