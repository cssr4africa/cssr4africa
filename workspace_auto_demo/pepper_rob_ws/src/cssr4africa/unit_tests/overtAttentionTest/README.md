<div align="center">
  <h1>Overt Attention Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `overtAttention` node within the CSSR4Africa project (`cssr_system` package). The node evaluate the execution of various attention modes on the Pepper humanoid robot. These tests ensure the accuracy and consistency of attention modes such as scanning, social, location, disabled, and seeking.
The package is compatible with the physical Pepper robots, enabling flexible testing scenarios. Configurations allow users to specify the specific modes to test, ensuring tailored and repeatable evaluations.
The tests are designed to verify proper integration and mode execution, ensuring that the robot performs attention accurately and naturally within its kinematic constraints. The results are logged in the file `~/workspace/pepper_rob_ws/src/unit_tests/overtAttentionTest/data/overtAttentionTestOutput.dat`.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.3 Overt Attention](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.3.pdf).

# Run the Overt Attention Unit Test 
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
   
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/overtAttentionTest/config/overtAttentionTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `scanning` | Test scanning mode | `true`, `false` |
   | `social` | Test social mode | `true`, `false` |
   | `seeking` | Test seeking mode | `true`, `false` |
   | `location` | Test location mode | `true`, `false` |
   | `disabled` | Test disabled mode | `true`, `false` |
   | `verboseMode` | Set the diagnostics message printing | `true`, `false` |

   <!-- - To run the tests on the physical platform, change the first line of `overtAttentionTestConfiguration.ini` file in the config folder to `platform robot`.  -->
   - To test any specific mode change the value to `true`, and for any mode to be excluded in the test change the value to `false`.
   - To set the diagnostics printing, change the last line to `verboseMode true`. The default is `true`.
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.3_Overt_Attention.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>. Otherwise, the preferred values are the ones already set in the `overtAttentionTestConfiguration.ini` file.</span>
  </div>

4. **Run the `overtAttentionTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests overtAttentionTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the overtAttentionTest (which launches the overtAttention node and run tests on it). This creates a driver for the `robotLocalization/pose` topic, `faceDetection/data` topic, and `soundDetection/direction` topic or lanches the actual nodes (if available).
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch unit_tests overtAttentionTestLaunchTestHarness.launch launch_drivers:=true launch_test:=true initial_robot_x:=<robot_x> initial_robot_y:=<robot_y> initial_robot_theta:=<robot_theta>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Setting the argument <code>launch_test</code> to <code>true</code> runs the tests based on the configuration. Setting the argument <code>launch_drivers</code> to <code>true</code> launches the drivers and stubs required to drive the <code>overtAttention</code> node from the <code>unit_tests</code> package, while setting it to <false> launches the actual nodes (if available) which include <code>faceDetection</code>, <code>soundDetection</code>, and <code>robotLocalization</code> from the <code>cssr_system</code> package. The default values are <true>. Ensure that the robot pose matches the exact pose of the robot in the world frame of reference.</span>
        </div>
<!-- 
## Simulator Robot
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
   
   Navigate to `~/workspace/pepper_sim_ws/src/unit_tests/overtAttentionTest/config/overtAttentionTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `scanning` | Test scanning mode | `true`, `false` |
   | `social` | Test social mode | `true`, `false` |
   | `seeking` | Test seeking mode | `true`, `false` |
   | `location` | Test location mode | `true`, `false` |
   | `disabled` | Test disabled mode | `true`, `false` |
   | `verboseMode` | Set the diagnostics message printing | `true`, `false` |

   - To run the tests on the physical platform, change the first line of `overtAttentionTestConfiguration.ini` file in the config folder to `platform robot`. 
   - To test any specific mode change the value to `true`, and for any mode to be excluded in the test change the value to `false`.
   - To set the diagnostics printing, change the last line to `verboseMode true`. The default is `true`.
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.3_Overt_Attention.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>. Otherwise, the preferred values are the ones already set in the `overtAttentionTestConfiguration.ini` file.</span>
  </div>

4. **Run the `overtAttentionTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests overtAttentionLaunchSimulator.launch
        ```

    - Open a new terminal to launch the overtAttentionTest (which launches the overtAttention node and run tests on it). This creates a driver for the `robotLocalization/pose` topic, `faceDetection/data` topic, and `soundDetection/direction` topic or launches the actual nodes (if available).
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && roslaunch unit_tests overtAttentionLaunchTestHarness.launch launch_drivers:=true launch_test:=true initial_robot_x:=<robot_x> initial_robot_y:=<robot_y> initial_robot_theta:=<robot_theta>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Setting the argument <code>launch_test</code> to <code>true</code> runs the tests based on the configuration. Setting the argument <code>launch_drivers</code> to <code>true</code> launches the drivers and stubs required to drive the <code>overtAttention</code> node from the <code>unit_tests</code> package, while setting it to <false> launches the actual nodes (if available) which include <code>faceDetection</code>, <code>soundDetection</code>, and <code>robotLocalization</code> from the <code>cssr_system</code> package. The default values are <true>. Ensure that the robot pose matches the exact pose of the robot in the world frame of reference.</span>
        </div> -->

## Tests Executed
### Location mode
The location mode tested moves the robot to look at two different locations in the room. The locations (x, y, z) currently set are:
  - `0 0 0`
  - `3 3 1`

The robot is expected to localize itself in the room (from the robotLocalization node) and look at these coordinates in the world frame of reference.

### Scanning mode
The put the attention in scanning mode and allows it to run in that mode for a minute.

### Social mode
The put the attention in social mode and allows it to run in that mode for a minute.

### Disabled mode
The put the attention in disabled mode for 25 seconds.

### Seeking mode
The put the attention in seeking mode and allows it to seek for mutual gaze until it timesout. It then compares the expected behavior with the behavior of the attention node and reports the result.


## Results
The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/overtAttentionTest/data/overtAttentionTestOutput.dat` file. It contains the test ran, the input commands and the satus of the test. Below is the output of the test when all tests are set to `true` in the `overtAttentionTestConfiguration.ini` file.
```
Overt Attention Test Report: robot
======================================
Date: 2025-01-10 12:16:06

Scanning Mode : 
	Result        : PASSED

Social Mode : 
	Result        : PASSED

Seeking Mode : 
	Status		  : MUTUAL GAZE DETECTED
	Result        : PASSED

Location Mode : 
	Result        : PASSED

Disabled Mode : 
	Result        : PASSED


```


## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, gesture types, debugging processes, and the overall functionality of the overtAttentionTest node, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.3_Overt_Attention.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:mdanso@andrew.cmu.edu">mdanso@andrew.cmu.edu</a><br>, <a href="mailto:aakinade@andrew.cmu.edu">aakinade@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>




## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-01-06