# Robot Localization Unit Test

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides comprehensive unit tests for the `robotLocalization` node within the CSSR4Africa project (`cssr_system` package). The test suite validates pose setting, pose reset functionality, marker detection, localization accuracy, service integration, and system stability using real robot hardware and ArUco markers.

The package implements testing of core localization functionalities, including triangulation and trilateration algorithms, service response validation, accuracy measurements, and robustness testing. The system leverages Google Test framework and integrates with the existing CSSR4Africa robotics infrastructure.

To accommodate diverse testing scenarios, parameters such as individual test categories, accuracy tolerances, and output verbosity are configurable. This package is designed for use with physical Pepper robots and integrates with Intel RealSense cameras for marker detection testing.

# Documentation
Accompanying this code is comprehensive test documentation that provides detailed explanation of the test architecture, implementation, and validation results. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Test

### Steps
1. **Install the required software components:**
   
   Set up the development environment for testing the Pepper robot localization system. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

   **Install Intel RealSense SDK and ROS Wrapper:**
   
   - Add Intel server to the list of repositories:
      ```bash
      sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
      sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
      ```
   
   - Install the Intel RealSense SDK 2.0 libraries:
      ```bash
      sudo apt update
      sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules
      sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
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
      cd ..
      catkin_make
      source devel/setup.bash 
      ```

3. **Update Test Configuration File:**
   
   Navigate to the test configuration file and update according to your testing needs:
   ```bash
   $HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotLocalizationTest/config/robotLocalizationTestConfiguration.ini
   ```

   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `poseSetTests` | Enable pose setting tests | `true`, `false` | `true` |
   | `poseResetTests` | Enable pose reset tests | `true`, `false` | `true` |
   | `markerDetectionTests` | Enable ArUco marker detection tests | `true`, `false` | `true` |
   | `accuracyTests` | Enable localization accuracy tests | `true`, `false` | `true` |
   | `serviceTests` | Enable service availability tests | `true`, `false` | `true` |
   | `stabilityTests` | Enable system stability tests | `true`, `false` | `true` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` | `true` |

4. **Set up ArUco Markers:**
   
   Place ArUco markers (DICT_4X4_100) in your environment according to the test landmark configuration:
   - Marker ID 1: Position (5.0, 4.8, 0.71)
   - Marker ID 2: Position (2.0, 5.4, 0.71) 
   - Marker ID 3: Position (2.6, 8.4, 0.71)
   - Additional markers as defined in `data/arucoLandmarks.json`

5. **Run the `robotLocalizationTest`:**

   > **NOTE:** To launch the RealSense camera when connected to a Jetson, refer to the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf) for instructions.

   **Approach 1: Test Execution using Launch Files (Recommended)**
   
   Run the complete test suite using two separate terminals:
    
   - **Terminal 1** - Launch robot interface (also launches the RealSense camera):
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        roslaunch unit_tests robotLocalizationTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```

      > **NOTE:** Ensure the IP addresses `robot_ip`, `roscore_ip`, and the `network_interface` are correctly set based on your robot's configuration and your computer's network interface. To launch using the default values that have been set in the launch file, simply run:

      ```bash
      roslaunch unit_tests robotLocalizationTestLaunchRobot.launch
      ```
      > If the Realsense camera is to be launched separately (e.g when connected and launched from a Jetson), simply pass the argument `realsense_camera:=false` in the launch command.

   - **Terminal 2** - Launch test harness (launches and run the tests):
        ```bash
        cd $HOME/workspace/pepper_rob_ws 
        source devel/setup.bash
        roslaunch unit_tests robotLocalizationTestLaunchTestHarness.launch
        ```
        > This command launches the robot localization node and the robot localization test node that runs all the tests with the physical robot. To run the test using drivers (i.e without the robot localization node and the physical robot), set the argument `launch_drivers:=true` in the launch command.

   **Approach 2: Manual Step-by-Step Execution**
   
   For individual component testing and debugging:
   
   - **Terminal 1** - Launch the Realsense Camera: 
        ```bash
        roslaunch realsense2_camera rs_camera.launch align_depth:=true
        ```
        > Ensure the Realsense camera is connected and installed, or follow the instructions in the Software Installation Manual as provided above when connected to a Jetson.

   - **Terminal 2** - Launch robot interface (bring up the robot): 
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        roslaunch unit_tests robotLocalizationTestLaunchRobot.launch realsense_camera:=false
        ```
        
   - **Terminal 3** - Start robot localization node:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun cssr_system robotLocalization
        ```
        
   - **Terminal 4** - Run localization tests:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun unit_tests robotLocalizationTest
        ```

## Test Categories and Coverage

### 1. Pose Setting Tests (`TestPoseSet`)
- **Purpose**: Validates the `/robotLocalization/set_pose` service functionality
- **Test Scenarios**:
  - Position 1 (2.0, 7.8, 270.0°)
  - Position 2 (2.6, 6.0, 180.0°)
  - Position 3 (5.0, 4.8, 90.0°)
  - Corner positions and edge cases
- **Success Criteria**: Service responds successfully and pose is set within tolerance

### 2. Pose Reset Tests (`TestPoseReset`)
- **Purpose**: Validates the `/robotLocalization/reset_pose` service and marker-based localization
- **Test Scenarios**:
  - Reset from known position
  - Marker detection and triangulation/trilateration
  - Pose correction accuracy
- **Success Criteria**: Service executes and pose resets.

### 3. Service Tests (`TestServices`)
- **Purpose**: Validates service availability and proper response formats
- **Test Scenarios**:
  - Service availability checks
  - Valid parameter handling
  - Response format validation
- **Success Criteria**: All services available and respond correctly

### 4. Accuracy Tests (`TestAccuracy`)
- **Purpose**: Measures localization precision and accuracy
- **Test Scenarios**:
  - High precision positions (±5cm, ±2°)
  - Medium distance positions
  - Near-landmark precision tests
- **Success Criteria**: Position error < 5cm, orientation error < 2°

### 5. Marker Detection Tests (`TestMarkerDetection`)
- **Purpose**: Validates ArUco markers detection
- **Test Scenarios**:
  - Single marker visibility
  - Multiple marker triangulation
- **Success Criteria**: Markers detected when visible, pose updates when possible

### 6. Stability Tests (`TestStability`)
- **Purpose**: Tests system robustness and edge case handling
- **Test Scenarios**:
  - Rapid successive pose sets
  - Edge case coordinates (boundaries, negative angles)
  - Service timeout resilience
- **Success Criteria**: System remains stable under stress conditions

## Test Results and Output

**Test Report Location:**
```bash
$HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotLocalizationTest/data/robotLocalizationTestOutput.dat
```

**Example Test Output:**
```
Robot Localization Test Report
==========================================
Date: 2025-01-15 14:30:22
Landmark File: arucoLandmarks.json
Topics File: pepperTopics.dat
Camera Info File: cameraInfo.json
Verbose Mode: true
Test Method: Service-based testing with pose verification

Pose Set Test 1: PASS
    Expected: (2, 7.8, 270)
    Actual: (2.01, 7.79, 269.8)
    Description: Home position

Pose Set Test 2: PASS
    Expected: (2.6, 6, 180)
    Actual: (2.58, 6.02, 180.3)
    Description: Office area

Service Availability Test: PASS
    Service: /robotLocalization/set_pose
    Arguments: N/A

Accuracy Test 1: PASS
    Position Error: 0.03 meters
    Orientation Error: 1.2 degrees

Marker Detection Test 1: PASS
    Marker ID: 1
    Detected: Yes

Stability Test: PASS
    Service: /robotLocalization/set_pose
    Arguments: Multiple rapid calls

==========================================
Test Execution Summary
==========================================
Total Tests: 5
Passed Tests: 5
Failed Tests: 0
Overall Result: ALL TESTS PASSED
Test report location: /home/user/workspace/pepper_rob_ws/src/.../robotLocalizationTestOutput.dat
```

## Service Commands for Manual Testing

**Set Robot Pose:**
```bash
rosservice call /robotLocalization/set_pose 2.0 7.8 270.0
```

**Reset Pose (Marker-based Localization):**
```bash
rosservice call /robotLocalization/reset_pose
```

**Monitor Pose Updates:**
```bash
rostopic echo /robotLocalization/pose
```

**View Marker Detection:**
```bash
rosrun image_view image_view image:=/robotLocalization/marker_image
```

## Troubleshooting

**Common Issues and Solutions:**

1. **Camera Not Detected:**
   ```bash
   # Verify RealSense camera
   realsense-viewer
   # Check camera topics
   rostopic list | grep camera
   ```

2. **Service Unavailable:**
   ```bash
   # Check robotLocalization node status
   rosnode list | grep robotLocalization
   # Restart if needed
   rosrun cssr_system robotLocalization
   ```

3. **Marker Detection Issues:**
   - Ensure proper lighting conditions
   - Check marker size and dictionary (DICT_4X4_100)
   - Verify marker positions match landmark file
   - Verify camera intrinsics are loaded

4. **Test Failures:**
   ```bash
   # Check test configuration
   cat ~/workspace/pepper_rob_ws/src/.../robotLocalizationTestConfiguration.ini
   # Review test report for specific failures
   cat ~/workspace/pepper_rob_ws/src/.../robotLocalizationTestOutput.dat
   ```

5. **Verification Commands:**
   ```bash
   # Check all required nodes
   rosnode list | grep -E "(robotLocalization|realsense)"
   
   # Monitor test logs
   rostopic echo /rosout | grep robotLocalizationTest
   
   # Verify camera info
   rostopic echo /camera/color/camera_info
   ```

## Test Performance Metrics

| Test Category | Expected Duration | Success Criteria |
|---------------|------------------|------------------|
| **Pose Set Tests** | ~15 seconds | All poses set within tolerance |
| **Pose Reset Tests** | ~20 seconds | Service responds, potential improvement |
| **Service Tests** | ~5 seconds | All services available |
| **Accuracy Tests** | ~25 seconds | Position error < 5cm, angle < 2° |
| **Marker Tests** | ~30 seconds | Detection when markers visible |
| **Stability Tests** | ~20 seconds | No system crashes or timeouts |

**Total Test Suite Duration:** ~2-3 minutes

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:ioj@alumni.cmu.edu">ioj@alumni.cmu.edu</a>, <a href="mailto:david@vernon.eu">david@vernon.eu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date: June 2025