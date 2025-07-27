<div align="center">
<h1> Face and Mutual Gaze Detection and Localization </h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Face and Mutual Gaze Detection and Localization** package is a ROS package designed to detect multiple faces and evaluate their **mutual gaze** in real-time by subscribing to an image topic. It publishes an array of detected faces and their mutual gaze status to the **/faceDetection/data** topic. Each entry in the published data includes the **label ID** of the detected face, the **centroid** coordinates representing the center point of each face, and a boolean value indicating **mutual gaze** status as either **true** or **false**, the **width** and **height** of the bounding box.

# 📄 Documentation
The main documentation for this deliverable is found in [D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf), which provides more details.

# 🛠️ Installation 

Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). This includes downloading the model files and putting them in the models files directory. Verify the model files are in the models directory. 
```sh
# Check if the model files are in the models directory:
ls ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models
```
If there is no output or the model directory isn't present, use the commands below to obtain the models:
```sh
# Install git lfs
sudo apt install -y git-lfs

# Clone the models from HuggingFace:
cd && git lfs install

git clone https://huggingface.co/cssr4africa/cssr4africa_models

# Create the models directory
mkdir -p ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models

# Move the models to the models directory:
mv ~/cssr4africa_models/face_detection/models/* ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models
```

## Installation on Ubuntu (x86-based Systems)

1. Pre-requisites  
Make sure you are running Ubuntu 20.04. The python environment to run face detection is similar to person detection. If you have set up the person detection Python environment, you can skip this step. If the intel real sense camera is used, make sure it uses USB 3.0. 

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
pip install -r  ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/face_detection_requirements_x86.txt
```


# 🔧 Configuration Parameters
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                      | Range/Values            | Default Value |
|-----------------------------|------------------------------------------------------------------|-------------------------|---------------|
| `algorithm`                 | Algorithm selected for face detection                            | `mediapipe`, `sixdrep`  | `sixdrep`     |
| `useCompressed`             | Use compressed ROS image topics                                  | `true`, `false`         | `true`        |
| `mpFacedetConfidence`       | Face detection confidence threshold (MediaPipe)                  | `[0.0 - 1.0]`           | `0.5`         |
| `mpHeadposeAngle`           | Head pose angle threshold in degrees (MediaPipe)                 | Positive integer        | `8`           |
| `centroidMaxDistance`       | Maximum centroid distance for centroid tracking                  | Positive integer        | `100`          |
| `centroidMaxDisappeared`    | Frames allowed before centroid tracker deregisters an object     | Positive integer        | `15`         |
| `sixdrepnetConfidence`      | Confidence threshold for face detection (SixDRepNet)             | `[0.0 - 1.0]`           | `0.90`        |
| `sixdrepnetHeadposeAngle`   | Head pose angle threshold in degrees (SixDRepNet)                | Positive integer        | `10`          |
| `sortMaxDisappeared`        | Maximum frames allowed for disappearance in SORT tracking        | Positive integer        | `5`          |
| `sortMinHits`               | Minimum consecutive detections to confirm object tracking (SORT) | Positive integer        | `3`          |
| `sortIouThreshold`          | IoU threshold for SORT tracker                                   | `[0.0 - 1.0]`           | `0.3`         |
| `imageTimeout`	            | Timeout (seconds) for shutting down the node after video ends	   | Float (seconds)	       | `5.0`         |
| `verboseMode`               | Enable visualization using OpenCV windows and detailed logging   | `true`, `false`         | `false`        |

> **Note:**  
> Enabling **`verboseMode`** (`true`) will activate real-time visualization via OpenCV windows. 

# 🚀 Running the node
**Run the `faceDetection` from the `cssr_system` package:**

Source the workspace in the first terminal:
  ```bash
  cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
  ```

Follow the steps below, run in different terminals.

  1️. Launch the robot and specify which camera to use. 
  ```bash
  roslaunch cssr_system face_detection_launch_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> camera:=<camera>
  ```

  The default camera is set to the RealSense.

  2️. Then run the Face and gaze detection and   Localization.

  In a new terminal, activate the Python environment. 
  ```bash
  # Activate the Python environment.
  source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_face_person_detection_env/bin/activate
  ```

  ```bash
  # Command to make the application executable.  
  chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/src/face_detection_application.py
  ```

  ```bash
  # Run the face_detection node
  rosrun cssr_system face_detection_application.py
  ```

#  🖥️ Output
The node publishes the detected faces and their corresponding centroid, the width and height of the bounding box, and a boolean array indicating whether a mutual gaze is established or not. When running in the verbose it display the OpenCv annotated color image and depth image that could help to visualize the result obtained. 

Subscription to the topic **faceDetection/data** allows verification of its publication status using the following command:

```bash
rostopic echo /faceDetection/data
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
