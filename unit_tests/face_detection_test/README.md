<div align="center">
  <h1>Face and Mutual Gaze Detection and Localization Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the **Face and Mutual Gaze Detection and Localization**, a node within the CSSR4Africa project (**cssr_system** package). The node evaluates the performance of face detection and mutual gaze under different conditions such as varying **lighting**, **occlusions**, and **different head poses**. It ensures that the detection algorithm remains robust across single and multiple face scenarios, maintaining accurate localization. The tests also assess the tracking stability of detected faces over multiple frames to measure consistency. Additionally, the module verifies that the mutual gaze detection correctly identifies when a user is engaging with the system, enabling natural human-robot interaction.

---

#  📄 Documentation
The main documentation for this deliverable is found in  **[D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf)**, which provides more details regarding how to run the test and gives detail about the face detection and localization module.  

---

#  Configuration File
The following table provides the key-value pairs used in the configuration file:

## 🔧 Configuration Parameters

| Parameter           | Description                                   | Possible Values            | Default Value |
|---------------------|-----------------------------------------------|----------------------------|---------------|
| `algorithm`         | Algorithm selected for face detection         | `mediapipe`, `sixdrep`     | `sixdrep`     |
| `useCompressed`     | Use compressed ROS image topics               | `true`, `false`            | `true`        |
| `saveVideo`         | Save the output video of the test             | `true`, `false`            | `true`       |
| `saveImage`         | Save individual image frames from the test    | `true`, `false`            | `true`       |
| `videoDuration`     | Duration (seconds) for saved video            | Positive integer           | `10`          |
| `imageInterval`     | Interval (seconds) between captured images    | Positive integer           | `5`           |
| `recordingDelay`    | Delay (seconds) before recording starts       | Positive integer           | `0`           |
| `maxFramesBuffer`   | Maximum number of frames to store in buffer   | Positive integer           | `300`         |
| `verboseMode`       | Enable detailed logging and visual output     | `true`, `false`            | `true`        |

---

> **Note:**  
> Enabling **`verboseMode`** (`true`) will activate detailed logging and **visualize outputs using OpenCV windows**.  

#  🚀 Run the Face Detection Unit Test
Before running the node, the configuration file must be set up correctly with the appropriate key-value pairs. The configuration file is typically located in the package folder under the `config` directory.

The test can be performed on a **physical robot using realsense or pepper camera** or using **pre-recorded video saved as a ROS bag file**. The specific test to run can be selected by modifying the configuration.

1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). You can use the README provided in the face detection node to install the necessary packages. 

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

3. **Clone and move the data files (if not already done)**:
  - Move to the home directory of the workspace and install git lfs
    ```bash 
        cd && git lfs install
      ```
  - Clone the `CSSR4Africa` data from the HuggingFace repository
    ```bash 
        git clone https://huggingface.co/cssr4africa/cssr4africa_unit_tests_data_files
      ```
  - Move the files to the data directory
    ```bash 
        mv cssr4africa_unit_tests_data_files/face_detection_test/data/* $HOME/workspace/pepper_rob_ws/src/unit_tests/face_detection_test/data
      ```

4. **Update Configuration File**
  Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/face_detection_test/config/face_detection_test_configuration.json` and update the configuration according to the key-value pairs as shown above.

5. **Launch the Physical Robot**
### Camera Input Options

- **`pepper`**: Use Pepper robot's built-in camera. If you choose **pepper** set the right **robot_ip** and **network_interface**. 
- **`realsense`**: Use Intel RealSense camera.
- **`video`**: Use a rosbag video recorded using the intel RealSense camera. 

```bash
# The camera in the launch file could be set as 'pepper', 'realsense' or 'video'
roslaunch face_detection_test_launch_robot.launch camera:=<camera> bag_file:=<bag_file> robot_ip:=<robot_ip> network_interface:=<network_interface>
```

### Bag File Options (`bag_file`):

- **`single_face`**: Contains data with a single face for basic detection testing and tracking with various distance form the robot.
- **`multiple_faces`**: Includes multiple faces to test detection robustness in crowded scenes.
- **`mutual_gaze`**: Data intended for analyzing gaze direction and mutual eye-contact scenarios.
- **`lighting_1`**: Features variations in lighting conditions to assess detection stability.
- **`lighting_2`**: Features variations in lighting conditions to 
assess detection performance.

<!-- > **Note:**  
> Before running the Test Harness, activate the face detection python environement. Refer the README.md file for the face detection node.
```bash
  source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_face_person_detection_env/bin/activate
``` -->
  ```bash
# Command to make application executable.  
chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/src/face_detection_application.py
chmod +x ~/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/face_detection_test/src/face_detection_test_application.py
```

6. **Run Face Detection Test Harness**
```bash
# The launch file launches the face_detection_node for the unit test.
roslaunch unit_tests face_detection_test_launch_test_harness.launch
```

# 🖥️ Output Result
When running the face detection test node, you will see:

- A window displaying the camera feed with face detection results  
Each detected face will have:

  - A colored bounding box (unique per face ID)  
  - Face ID label  
  - "Engaged" or "Not Engaged" status based on mutual gaze  
  - Depth information showing distance from camera  

The details of the result is documented in the Face and Mutual Gaze Detection and Localization Deliverible Report.

# 💡Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

# 📜 License
  Copyright (C) 2023 CSSR4Africa Consortium  
  Funded by the African Engineering and Technology Network (Afretec)  
  Inclusive Digital Transformation Research Grant Programme 

  2025-03-15