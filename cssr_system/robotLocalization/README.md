<div align="center">
  <h1>Robot Localization</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotLocalization` ROS node provides accurate pose estimation capabilities for the Pepper humanoid robot using visual landmark detection and sensor fusion. This node combines ArUco marker detection (from RGB and depth cameras) with odometry data to deliver robust pose estimation in indoor environments.

The package implements both triangulation (RGB) and trilateration (RGB-D) algorithms for absolute pose computation from detected ArUco landmarks, then maintains continuous positioning through odometry integration. The system supports periodic automatic pose correction and on-demand pose reset services for reliable localization in dynamic environments.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of this node and its software. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Node

## Physical Robot 

### Prerequisites

1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

   **Install Intel RealSense SDK and ROS Wrapper (For the Intel RealSense camera):**
   
   - Add Intel server to the list of repositories:
      ```bash
      sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
      sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
      ```
   
   - Install the Intel RealSense SDK 2.0 libraries and utilities:
      ```bash
      sudo apt update
      sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules
      ```
   
   - Install the ROS wrapper for RealSense cameras:
      ```bash
      # For ROS Noetic:
      sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
      ```
   
   - Verify the installation by connecting your RealSense camera and running:
      ```bash
      realsense-viewer
      ```  

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
         cd .. && source devel/setup.bash && catkin_make
       ```

3. **Install ArUco Library Package (ROS Noetic)**
     ```bash
        sudo apt update
        sudo apt install ros-noetic-aruco ros-noetic-aruco-msgs ros-noetic-aruco-ros
      ``` 

## Configuration

### JSON Configuration File

Navigate to the configuration file and update it according to your use case and environment setup:

**Main Configuration** (`config/robotLocalizationConfiguration.json`):

| Parameter | Description | Values | Default |
|-----------|-------------|---------|---------|
| `verboseMode` | Diagnostic info printing | `true`, `false` | `true` |
| `camera` | RGB camera source | `"RGBRealSense"`, `"FrontCamera"` | `"RGBRealSense"` |
| `depthCamera` | Depth camera source | `"DepthRealSense"` | `"DepthRealSense"` |
| `useDepth` | Enable depth-based trilateration | `true`, `false` | `false` |
| `resetInterval` | Automatic pose reset interval (seconds) | Float | `30.0` |
| `absolutePoseTimeout` | Timeout for pose validity (seconds) | Float | `300.0` |
| `cameraInfoTimeout` | Timeout for retrieving camera intrinsics (seconds) | Float | `15.0` |
| `useHeadYaw` | Compensate for head rotation | `true`, `false` | `true` |
| `headYawJointName` | Name of head yaw joint | String | `"HeadYaw"` |
| `mapFrame` | Map coordinate frame | String | `"map"` |
| `odomFrame` | Odometry coordinate frame | String | `"odom"` |

### Data Files Configuration

**Landmark (ArUco Markers) Data** (`data/arucoLandmarks.json`): 
These are the positions of ArUco markers placed in the known environment. Update accordingly.
```json
{
  "landmarks": [
    {
      "id": 1,
      "x": 5.0,   // X coordinate in meters
      "y": 4.8,   // Y coordinate in meters  
      "z": 0.71   // Z coordinate in meters (ideal is camera height)
    },
    {
      "id": 2,
      "x": 2.0,
      "y": 5.4,
      "z": 0.71
    }
  ]
}
```

**Camera Calibration** (`data/cameraInfo.json`):
Default calibration if camera intrinsics are not retrieved automatically from camera. Update accordingly.
```json
{
  "cameraInfo": {
    "fx": 911.6033325195312,   // Focal length X
    "fy": 910.8851318359375,   // Focal length Y
    "cx": 655.0755615234375,   // Principal point X
    "cy": 363.9165954589844    // Principal point Y
  }
}
```

To retrieve and update camera intrinsic values:
- Launch the RealSense camera
  ```bash
  roslaunch realsense2_camera rs_camera.launch
  ```
- Echo the camera info topic and retrieve values
  ```bash
  rostopic echo /camera/color/camera_info
  ```

**Topic Mapping** (`data/pepperTopics.dat`):
Maps logical sensor names to actual ROS topics.
```
FrontCamera         /naoqi_driver/camera/front/image_raw
StereoCamera        /naoqi_driver/camera/stereo/image_raw
RGBRealSense        /camera/color/image_raw
DepthRealSense      /camera/aligned_depth_to_color/image_raw
CameraInfo          /camera/color/camera_info
Odometry            /naoqi_driver/odom
IMU                 /naoqi_driver/imu/base
HeadYaw             /joint_states
```

## ArUco Marker Setup

Place ArUco markers (DICT_4X4_100) in your physical environment at the coordinates specified in `arucoLandmarks.json`. Ensure markers are:
- Clearly visible from robot operating areas
- At appropriate heights (recommended: camera height ±0.5m)
- Well-distributed to avoid collinear configurations and acute angles between markers
- Properly lit and unobstructed
- Using the DICT_4X4_100 dictionary for compatibility

## Running the System

### Launch Commands

Follow these steps, running each command in different terminals:

 > **NOTE:** To launch the RealSense camera when connected to a Jetson, refer to the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf) for instructions.

1. **Source the workspace in first terminal:**
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
   ```

2. **Launch the robot:**
   ```bash
   roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
   ```
   
   > **NOTE:** Ensure that the IP addresses `robot_ip`, `roscore_ip` and the network interface `network_interface` are correctly set based on your robot's configuration and your computer's network interface. To launch using the default values that have been set in the launch file, simply run:

      ```bash
      roslaunch cssr_system cssrSystemLaunchRobot.launch
      ```

3. **Launch the robotLocalization node:**
   
   **Option A: Full launch RealSense camera and robotLocalization node:**
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system robotLocalizationLaunchRobot.launch
   ```
   
   **Option B: Run RealSense camera and robotLocalization node separately**:
   > Run RealSense camera
   ```bash
   roslaunch realsense2_camera rs_camera.launch align_depth:=true
   ```
   > Run robotLocalization node
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
   ```

### Required Topics

The `robotLocalization` node requires the following topics to be available:
- `/camera/color/image_raw` - RealSense RGB camera images
- `/camera/aligned_depth_to_color/image_raw` - RealSense Depth images (if using depth mode)
- `/camera/color/camera_info` - RealSense Camera intrinsic parameters
- `/naoqi_driver/odom` - Robot odometry
- `/joint_states` - Joint positions (for head yaw)

## Service Interface

Upon launching the node, the following services are available:
- `/robotLocalization/set_pose`
- `/robotLocalization/reset_pose`

```bash
# Verify services are running
rosservice list | grep /robotLocalization
```

### Service Commands

**Set the Initial Robot Pose (for manually setting the robot position):**
```bash
rosservice call /robotLocalization/set_pose x y theta
```
- `x`: X-coordinate in meters (float)
- `y`: Y-coordinate in meters (float)  
- `theta`: Orientation in degrees (float)

**Reset Pose Using Landmarks:**
```bash
rosservice call /robotLocalization/reset_pose
```
- Triggers automatic pose computation from detected markers

**View Detected Markers:**
```bash
rosrun image_view image_view image:=/robotLocalization/marker_image
```

**Echo the Pose Topic:**
```bash
rostopic echo /robotLocalization/pose
```

### Example Service Calls

- Set robot at position (2.0, 6.6) facing 0 degrees:
  ```bash
  rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
  ```
- Set robot facing -45 degrees (315 degrees):
  ```bash
  rosservice call /robotLocalization/set_pose 2.0 6.6 315.0
  ```
- Trigger pose reset from landmarks:
  ```bash
  rosservice call /robotLocalization/reset_pose
  ```

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/robotLocalization/pose` | `geometry_msgs/Pose2D` | Current robot pose (x, y, theta in degrees) |
| `/robotLocalization/marker_image` | `sensor_msgs/Image` | Annotated image showing detected ArUco markers |

## Algorithm Details

### Triangulation Algorithm (RGB-only mode)
- Detects ArUco markers in camera image using OpenCV ArUco library
- Computes viewing angles between marker pairs using camera intrinsics
- Uses geometric triangulation with circle-circle intersection to determine robot position
- Employs multi-solution scoring system based on geometric constraints:
  - Triangle area quality (larger areas = better precision)
  - Distance reasonableness (prefers moderate distances)
  - Angle quality (avoids degenerate triangulation)
  - Landmark separation validation
  - Proximity penalties for unrealistic positions

### Trilateration Algorithm (Depth mode)
- Combines ArUco detection with depth measurements from RGB-D camera
- Extracts distance information from depth image at marker centers
- Solves system of distance equations using circle intersection
- Uses least-squares approach for overdetermined systems

### Pose Fusion
- Maintains baseline pose from absolute measurements
- Integrates odometry for continuous pose updates between landmark detections
- Applies periodic corrections to prevent drift accumulation
- Supports head yaw compensation for camera orientation changes

## Monitoring and Debugging

**Enable Verbose Output:**
```bash
# Set parameter at runtime
rosparam set /robotLocalization/verbose true
```
or modify the configuration file

**Check Node Status:**
```bash
rosnode info /robotLocalization
```

**Monitor Topic Rates:**
```bash
rostopic hz /robotLocalization/pose
rostopic hz /camera/color/image_raw
```

**Monitor Transforms:**
```bash
rosrun tf tf_echo map odom
```

## Troubleshooting

### Common Issues

1. **No markers detected:**
   - Check marker visibility and lighting conditions
   - Verify landmark coordinates in config file match physical placement
   - Ensure markers use DICT_4X4_100 dictionary
   - Check camera image quality with `rosrun image_view image_view image:=/camera/color/image_raw`

2. **Inaccurate pose estimation:**
   - Calibrate camera intrinsics properly using RealSense calibration tools
   - Check for marker ID conflicts in landmark configuration
   - Verify landmark placement accuracy with measuring tools
   - Avoid collinear marker configurations and acute viewing angles

3. **High pose drift:**
   - Check odometry quality with `rostopic echo /naoqi_driver/odom`
   - Ensure sufficient landmark coverage throughout operating area
   - Verify transforms between coordinate frames

4. **Camera intrinsics not received:**
   - Check RealSense camera connection and drivers
   - Verify camera info topic is publishing: `rostopic echo /camera/color/camera_info`
   - Ensure camera launch includes intrinsic calibration
   - Fallback configuration will be loaded from file after timeout

5. **Service call failures:**
   - Verify node is running: `rosnode list | grep robotLocalization`
   - Check service availability: `rosservice list | grep robotLocalization`
   - Ensure minimum 3 markers are visible for pose reset

### Performance Optimization

- **For better accuracy:** Use RGB mode (`useDepth: false`)
- **For faster processing:** Disable verbose mode and reduce image resolution
- **For stable operation:** Place markers at consistent heights and ensure good lighting

## System Requirements

- **ROS:** Noetic
- **OpenCV:** 4.2+ with ArUco support
- **Camera:** Intel RealSense D435/D455 or compatible RGB-D camera
- **Markers:** ArUco markers from DICT_4X4_100 dictionary
- **Compute:** Sufficient processing power for real-time image processing

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: [ioj@alumni.cmu.edu](mailto:ioj@alumni.cmu.edu), [david@vernon.eu](mailto:david@vernon.eu)
- Visit: [www.cssr4africa.org](http://www.cssr4africa.org)

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date: 2025-06-23