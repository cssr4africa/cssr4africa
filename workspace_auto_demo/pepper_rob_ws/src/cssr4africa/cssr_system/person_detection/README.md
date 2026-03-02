<div align="center">
<h1> Person Detection and Localization </h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Person Detection and Localization** package is a ROS package designed to detect multiple persons in real-time by subscribing to color and depth image topics. It publishes an array of detected persons to the **/personDetection/data** topic. Each entry in the published data includes the **label ID** of the detected person, the **centroid** coordinates representing the center point of each person, the **width** and **height** of the bounding box, and the **depth** information in meters.

# 📄 Documentation
The main documentation for this deliverable, found in [D4.2.1 Person Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.1.pdf), provides more details about the implementation and usage of the person detection system.

## Installation on Ubuntu (x86-based Systems)
Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). This includes downloading the model files and putting them in the models files directory. Verify the model files are in the models directory. 
```sh
# Check if the model files are in the models directory:
ls ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models
```
If there is no output, use the commands below to obtain the models:
```sh
# Install git lfs
sudo apt install -y git-lfs

# Clone the models from HuggingFace:
cd && git lfs install

git clone https://huggingface.co/cssr4africa/cssr4africa_models

# Create the models directory
mkdir -p ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models

# Move the models to the models directory:
mv ~/cssr4africa_models/person_detection/models/* ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models
```

1. Prerequisites
Make sure you are running Ubuntu 20.04. If you have set up the Python environment using the ``face detection`` README, you can skip this step. If the Intel RealSense camera is used, make sure it uses USB 3.0. 

2. Install Python 3.10 and Virtual Environment.
```sh
# Update system packages
sudo apt update && sudo apt upgrade -y

# Add the deadsnakes PPA for Python versions
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update

# Install Python 3.10
sudo apt install python3.10 python3.10-venv python3.10-distutils -y

# Verify Python installation
python3.10 --version
```

3. Set Up Virtual Environment
```sh
# Create a virtual environment:
mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
cd $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
python3.10 -m venv cssr4africa_face_person_detection_env

# Activate the virtual environment:
source cssr4africa_face_person_detection_env/bin/activate

# Upgrade pip in the virtual environment:
pip install --upgrade pip
```

4. Install Required Packages
```sh
# Install PyTorch with CUDA support:
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install additional requirements:
pip install -r  ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/person_detection_requirements_x86.txt
```

# 🔧 Configuration Parameters
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                      | Range/Values            | Default Value |
|-----------------------------|------------------------------------------------------------------|-------------------------|---------------|
| `useCompressed`             | Use compressed ROS image topics                                  | `true`, `false`         | `true`       |
| `confidenceThreshold`       | Confidence threshold for person detection                        | `[0.0 - 1.0]`           | `0.8`         |
| `sortMaxDisappeared`        | Maximum frames allowed for disappearance in SORT tracking        | Positive integer        | `30`          |
| `sortMinHits`               | Minimum consecutive detections to confirm object tracking (SORT) | Positive integer        | `20`           |
| `sortIouThreshold`          | IoU threshold for SORT tracker                                   | `[0.0 - 1.0]`           | `0.3`         |
| `imageTimeout`	            | Timeout (seconds) for shutting down the node after video ends	   | Float (seconds)	       | `5.0`         |
| `verboseMode`               | Enable visualization using OpenCV windows and detailed logging   | `true`, `false`         | `false`       |

> **Note:**  
> Enabling **`verboseMode`** (`true`) will activate real-time visualization via OpenCV windows. 

# 🚀 Running the node
**Run the `personDetection` from the `cssr_system` package:**

Source the workspace in the first terminal:
  ```bash
  cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
  ```

Follow the steps below, run in different terminals.

  1️. Launch the robot and specify which camera to use. 

  > If you are using Pepper's camera, you need to specify the robot_ip, roscore_ip, and network_interface, and specify in the camera `pepper`. If you are using the Intel RealSense, just specify the camera as `realsense`. 

  ```bash
  roslaunch cssr_system person_detection_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> camera:=<camera>
  ```
  The default camera is set to the RealSense.

  2️. Then run the Person Detection and Localization.

  In a new terminal, activate the Python environment. 
  ```bash
  source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_face_person_detection_env/bin/activate
  ```

  ```bash
  # Command to the application.py executable.
  chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/src/person_detection_application.py
  ```

  ```bash
  # Run the person_detection node
  rosrun cssr_system person_detection_application.py
  ```

#  🖥️ Output
The node publishes the detected persons and their corresponding centroid, the width and height of the bounding box, and the depth information in meters. When running in verbose mode, it displays the OpenCV annotated color image and depth image, which helps visualize the results obtained.

Subscription to the topic **personDetection/data** allows verification of its publication status using the following command:

```bash
rostopic echo /personDetection/data
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

2025-07-27
