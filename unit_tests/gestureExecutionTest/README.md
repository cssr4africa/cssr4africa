
<div align="center">
  <h1>Gesture Execution Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `gestureExecution` node within the CSSR4Africa project (`cssr_system` package). The node evaluate the execution of various gestures on the Pepper humanoid robot. These tests ensure the accuracy and consistency of gestures such as deictic (pointing), iconic, bow, and nod gestures.
The package is compatible with the physical Pepper robots, enabling flexible testing scenarios. Configurations allow users to specify the specific gestures to test, ensuring tailored and repeatable evaluations.
The tests are designed to verify proper integration and motion execution, ensuring that the robot performs gestures accurately and naturally within its kinematic constraints. The results are logged in the file `~/workspace/pepper_rob_ws/src/unit_tests/gestureExecutionTest/data/gestureExecutionTestOutput.dat` for the physical robot.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.5.1 Gesture Execution](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf).

# Run the Gesture Execution Unit Test 
## Physical Robot 
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

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
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/gestureExecutionTest/config/gestureExecutionTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `iconic` | Test iconic gestures | `true`, `false` |
   | `deictic` | Test deictic gestures | `true`, `false` |
   | `bow` | Test bow gestures | `true`, `false` |
   | `nod` | Test nod gestures | `true`, `false` |
   | `symbolic` | Test symbolic gestures | `true`, `false` |
   | `verboseMode` | Set the diagnostics message printing | `true`, `false` |

   <!-- - To execute the gestures on the physical platform, change the first line of `gestureExecutionTestConfiguration.ini` file in the config folder to “`platform robot`”.  -->
   - To test any specific gesture, change the value to `true` and for any gesture to be excluded in the test, change the value to `false`.
   - To set the diagnostics printing, change the last line to "`verboseMode true`. The default is `true`.
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>. Otherwise, the preferred values are the ones already set in the `gestureExecutionTestConfiguration.ini` file.</span>
  </div>

4. **Run the `gestureExecutionTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests gestureExecutionTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the gestureExecutionTest (which launches the gestureExecution node and run tests on it). This creates a driver for the `robotLocalization/pose` topic and a stub for the `overtAttention/set_mode` service or launches the actual nodes (if available).
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch unit_tests gestureExecutionTestLaunchTestHarness.launch launch_drivers_stubs:=true launch_test:=true initial_robot_x:=<robot_x> initial_robot_y:=<robot_y> initial_robot_theta:=<robot_theta>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Setting the argument <code>launch_test</code> to <code>true</code> runs the tests based on the configuration. Setting the argument <code>launch_drivers_stubs</code> to <code>true</code> launches the drivers and stubs required to drive the <code>gestureExecution</code> node from the <code>unit_tests</code> package, while setting it to <false> launches the actual nodes (if available) which include <code>faceDetection</code>, <code>soundDetection</code>, <code>overtAttention</code>, and <code>robotLocalization</code> from the <code>cssr_system</code> package. The default values are <true>. Ensure that the robot pose matches the exact pose of the robot in the world frame of reference.</span>
        </div>
<!-- 
## Simulator Robot -- 
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 


2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_sim_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && catkin_make && source devel/setup.bash
       ```
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_sim_ws/src/unit_tests/gestureExecutionTest/config/gestureExecutionTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `iconic` | Test iconic gestures | `true`, `false` |
   | `deictic` | Test deictic gestures | `true`, `false` |
   | `bow` | Test bow gestures | `true`, `false` |
   | `nod` | Test nod gestures | `true`, `false` |
   | `symbolic` | Test symbolic gestures | `true`, `false` |
   | `verboseMode` | Set the diagnostics message printing | `true`, `false` |

   - To execute the gestures on the physical platform, change the first line of `gestureExecutionTestConfiguration.ini` file in the config folder to “`platform robot`”. 
   - To test any specific gesture, change the value to `true` and for any gesture to be excluded in the test, change the value to `false`.
   - To set the diagnostics printing, change the last line to "`verboseMOde true`. The default is `true`.

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>. Otherwise, the preferred values are the ones already set in the `gestureExecutionTestConfiguration.ini` file.</span>
  </div>

4. **Run the `gestureExecutionTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests gestureExecutionLaunchSimulator.launch
        ```

    - Open a new terminal to launch the gestureExecutionTest (which launches the gestureExecution node and run tests on it). This creates a driver for the `robotLocalization/pose` topic and a stub for the `overtAttention/set_mode` service or launches the actual nodes (if available).
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && roslaunch unit_tests gestureExecutionLaunchTestHarness.launch launch_drivers_stubs:=true launch_test:=true initial_robot_x:=<robot_x> initial_robot_y:=<robot_y> initial_robot_theta:=<robot_theta>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Setting the argument <code>launch_test</code> to <code>true</code> runs the tests based on the configuration. Setting the argument <code>launch_drivers_stubs</code> to <code>true</code> launches the drivers and stubs required to drive the <code>gestureExecution</code> node from the <code>unit_tests</code> package, while setting it to <false> launches the actual nodes (if available) which include <code>faceDetection</code>, <code>soundDetection</code>, <code>overtAttention</code>, and <code>robotLocalization</code> from the <code>cssr_system</code> package. The default values are <true>. Ensure that the robot pose matches the exact pose of the robot in the world frame of reference.</span>
        </div> -->

## Tests Executed
### Deictic Gestures
The deictic gestures tested moves the robot to point at four different locations in the room. The locations (x, y, z) currently set are:
  - `0.6 5.1 0.82`
  - `6.8 4.8 0.82`
  - `5.0 1.8 0.82`
  - `6.8 9.6 0.82`

The robot is expected to localize itself in the room (from the robotLocalization node) and point towards these coordinates in the world frame of reference.

### Iconic Gestures
The iconic gestures executed are the `welcome` gesture, `wave` gesture and the `handshake` gesture. The welcome gesture is identified with either ID `01` or `02`, the wave gesture is identified with ID `03`, while the handshake gesture is identified with ID `04` or `05`. The robot is expected to open its arms as a sign of welcome, wave its right arm and stretch its right arm while attemting to support with the left hand in a handshake gesture.

### Bow Gesture
The bow gestures is executed to move the robot's torso in a bow motion. The robot bows at different degrees. The degrees are:
- `15`
- `30`
- `45`

### Nod Gesture
The nod gestures is executed to move the robot's head in a nod motion. The robot nods at different degrees. The degrees are:
- `15`
- `30`
- `45`


## Results
The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/gestureExecutionTest/data/gestureExecutionTestOutput.dat` file for the physical robot. It contains the test ran, the input commands and the satus of the test. Below is the output of the test when all tests are set to `true` in the `gestureExecutionTestConfiguration.ini` file.
```
Gesture Execution Test Report: robot
======================================
Date: 2024-12-06 14:03:22

Iconic Gesture 01 : Welcome Gesture
	Joint Names   :	LShoulderPitch LShoulderRoll LElbowYaw LElbowRoll LWristYaw 
	Joint Angles  :	1.7625 0.09970 -1.7150 -0.1334 0.06592 
					1.047197 0.2618 -1.5708 -0.0087 -1.047198 
					1.7625 0.09970 -1.7150 -0.1334 0.06592 
	Joint Names   :	RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw 
	Joint Angles  :	1.7410 -0.09664 1.6981 0.09664 -0.05679 
					1.047197 -0.2618 1.5708 0.0087 1.047198 
					1.7410 -0.09664 1.6981 0.09664 -0.05679 
	Result        : PASSED

Iconic Gesture 02 : Welcome Gesture
	Joint Names   :	LShoulderPitch LShoulderRoll LElbowYaw LElbowRoll LWristYaw 
	Joint Angles  :	1.7625 0.09970 -1.7150 -0.1334 0.06592 
					1.047197 0.2618 -1.5708 -0.0087 -1.047198 
					1.7625 0.09970 -1.7150 -0.1334 0.06592 
	Joint Names   :	RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw 
	Joint Angles  :	1.7410 -0.09664 1.6981 0.09664 -0.05679 
					1.047197 -0.2618 1.5708 0.0087 1.047198 
					1.7410 -0.09664 1.6981 0.09664 -0.05679 
	Result        : PASSED

Iconic Gesture 03 : Wave Right Hand
	Joint Names   :	RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw 
	Joint Angles  :	1.7410 -0.09664 1.6981 0.09664 -0.05679 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0 
					0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0 
					1.7410 -0.09664 1.6981 0.09664 -0.05679 
	Result        : PASSED

Iconic Gesture 04 : Shake Gesture
	Joint Names   :	LShoulderPitch LShoulderRoll LElbowYaw LElbowRoll LWristYaw 
	Joint Angles  :	1.7625 0.09970 -1.7150 -0.1334 0.06592 
					0.358 0.15 0.066 -1.333 -1.88 
	Joint Names   :	RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw 
	Joint Angles  :	1.7410 -0.09664 1.6981 0.09664 -0.05679 
					0.183 -0.02 -0.082 0.5 1.634 
	Result        : PASSED

Iconic Gesture 05 : Shake Gesture
	Joint Names   :	LShoulderPitch LShoulderRoll LElbowYaw LElbowRoll LWristYaw 
	Joint Angles  :	1.7625 0.09970 -1.7150 -0.1334 0.06592 
					0.358 0.15 0.066 -1.333 -1.88 
	Joint Names   :	RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw 
	Joint Angles  :	1.7410 -0.09664 1.6981 0.09664 -0.05679 
					0.183 -0.02 -0.082 0.5 1.634 
	Result        : PASSED

Deictic Gesture 
	To			  : 0.6 5.1 0.82
	Result        : PASSED

Deictic Gesture 
	To			  : 6.8 4.8 0.82
	Result        : PASSED

Deictic Gesture 
	To			  : 5.0 1.8 0.82
	Result        : PASSED

Deictic Gesture 
	To			  : 6.8 9.6 0.82
	Result        : PASSED

Bow Gesture 
	Degree		  : 15
	Result        : PASSED

Bow Gesture 
	Degree		  : 30
	Result        : PASSED

Bow Gesture 
	Degree		  : 45
	Result        : PASSED

Nod Gesture 
	Degree		  : 15
	Result        : PASSED

Nod Gesture 
	Degree		  : 30
	Result        : PASSED

Nod Gesture 
	Degree		  : 45
	Result        : PASSED
```


## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, gesture types, debugging processes, and the overall functionality of the gestureExecutionTest node, please refer to the <a href="hhttps://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:aakinade@andrew.cmu.edu">aakinade@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>




## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2024-01-10