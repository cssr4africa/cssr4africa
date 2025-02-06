<div align="center">
  <h1>Gesture Execution</h1>
</div>


<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `gestureExecution` ROS node enhances the Pepper humanoid robot's ability to perform meaningful and dynamic gestures, improving its interactive capabilities. This ROS node hosts a service that allows users to invoke a variety of gestures, including deictic (pointing), iconic, symbolic, nod, and bow gestures.
The package leverages biological motion profile to ensure natural and fluid execution, making interactions more engaging and lifelike. Gestures are executed using the robot's built-in kinematic capabilities, and the system ensures that the motions are aligned with the robot's physical constraints.
To accommodate diverse interaction scenarios, gesture parameters such as duration, bow/nod degree and poinitng location are configurable, offering flexibility in tailoring the gestures to specific use cases. This package is designed for use with the physical Pepper robots, allowing seamless integration into larger robotics applications through ROS topic and service interfaces.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.5.1 Gesture Execution](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf).


 
# Run the Gesture Execution Node
## Physical Robot 
### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

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
   
   Navigate to the configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/gestureExecution/config/gestureExecutionConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `interpolation` | Motion profile | `biological`, `linear` |
   | `gestureDescriptors` | Gesture Descriptors filename | `gestureDescriptors.dat` |
   | `simulatorTopics` | Simulator robot topic mapping file | `simulatorTopics.dat` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` |

   <!-- - To execute the gestures on the physical platform, change the first line of `gestureExecutionConfiguration.ini` file in the config folder to “`platform robot`”.  -->
   - To execute gestures using a biological motion model, change the second line of `gestureExecutionConfiguration.ini` file in the config folder to "`interpolation biological`". If the linear velocity control mechanism is preferred, change the second line of the file to "`interpolation linear`"
   - In order to execute iconic gestures, the gesture descriptors are defined in a file (located in the data folder). This file is specified by the third line of the config file as "`gestureDescriptors gestureDescriptors.dat`".	
   - All actuators have a topic, through which actiuation is carried out. These topics are specified in a key-value pair format in a file which is defined by the fourth and fifth line of the `gestureExecutionConfiguration.ini` file for the simulator and physical robot respectively. These files are found in the data folder. The files are specified as "`simulatorTopics simulatorTopics.dat`" and "`pepperTopics pepperTopics.dat`".	
   - The system is capable of printing diagnostic informqation to the terminal. This behaviour is controlled by the sixth key-value pair in the `gestureExecutionConfiguration.ini` file. To set diagnostic messages to printing, change the sixth line to "`verboseMode true`". If no diagnostic printing is required, change the sixth line to "`verboseMode false`"

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>

4. **Run the `gestureExecution` from the `cssr_system`  package:**
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
         roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
            <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
            <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
         </div>
    - Open a new terminal to launch the `gestureExecution` node.
        ```bash
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system gestureExecution
        ```

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>gestureExecution</code> node requires the <code>/overtAttention/set_mode</code> service and the <code>robotLocalization/pose</code> topic to be available, which can be hosted by running the <code>overtAttention</code> and <code>robotLocalization</code> node in the <code>cssr_system</code> package or running the <code>gestureExecutionStub</code> and <code>gestureExecutionDriver</code> in the <code>unit_tests</code> package before running the <code>gestureExecution</code>  node: </span>

         - <span style="color: #cccccc; font-weight: bold">(Option 1A):</span>  Run the <code>overtAttention</code> node of the <code>cssr_system</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system overtAttention
            ``` 
         - <span style="color: #cccccc; font-weight: bold">(Option 1B):</span> Run the <code>robotLocalization</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2A): </span> Run the <code>gestureExecutionStub</code> of the <code>unit_tests</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun unit_tests gestureExecutionStub
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2B): </span> Run the <code>gestureExecutionDriver</code> of the <code>unit_tests</code> package (in a new terminal) passing the robot pose (robot_x, robot_y, robot_theta) as arguments:
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun unit_tests gestureExecutionDriver <robot_x> <robot_y> <robot_theta>
            ```
            <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
               <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
               <span style="color: #cccccc;">Ensure that the robot pose <code>robot_x</code>, <code>robot_y</code> and <code>robot_theta</code> are correctly set based on your robot's position in the world. If these arguments are not supplied, the robot assumes its position as origin <code>(0, 0, 0)</code> </span>
            </div>
         </div>
      
<!-- 
## Simulator Robot

### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in the simulated environment. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

2. **Clone and build the project (if not already cloned):**
   - Move to the source directory of the workspace:
      ```bash
      cd $HOME/workspace/pepper_sim_ws/src
      ```
   - Clone the `CSSR4Africa` software from the GitHub repository:
      ```bash
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files:
      ```bash 
      cd .. && catkin_make && source devel/setup.bash 
       ```

3. **Update Configuration File:**
   
   Navigate to the configuration file located at `$HOME/workspace/pepper_sim_ws/src/cssr4africa/gestureExecution/config/gestureExecutionConfiguration.ini`and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `interpolation` | Motion profile | `biological`, `linear` |
   | `gestureDescriptors` | Gesture Descriptors filename | `gestureDescriptors.dat` |
   | `simulatorTopics` | Simulator robot topic mapping file | `simulatorTopics.dat` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` |

   - To execute the gestures on the simulator platform, change the first line of the file to “`platform simulator`”.
   - To execute gestures using a biological motion model, change the second line of `gestureExecutionConfiguration.ini` file in the config folder to "`interpolation biological`". If the linear velocity control mechanism is preferred, change the second line of the file to "`interpolation linear`"
   - In order to execute iconic gestures, the gesture descriptors are defined in a file (located in the data folder). This file is specified by the third line of the config file as "`gestureDescriptors gestureDescriptors.dat`".	
   - All actuators have a topic, through which actiuation is carried out. These topics are specified in a key-value pair format in a file which is defined by the fourth and fifth line of the `gestureExecutionConfiguration.ini` file for the simulator and physical robot respectively. These files are found in the data folder. The files are specified as "`simulatorTopics simulatorTopics.dat`" and "`pepperTopics pepperTopics.dat`".	
   - The system is capable of printing diagnostic informqation to the terminal. This behaviour is controlled by the sixth key-value pair in the `gestureExecutionConfiguration.ini` file. To set diagnostic messages to printing, change the sixth line to "`verboseMode true`". If no diagnostic printing is required, change the sixth line to "`verboseMode false`"

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>


4. **Run the `gestureExecution` from the `cssr_system` package:**:
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
         cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the simulator robot:
        ```bash
         roslaunch cssr_system cssrSystemLaunchSimulator.launch
        ```
    - Open a new terminal to launch the `gestureExecution` node.
        ```bash
         cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system gestureExecution
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>gestureExecution</code> node requires the <code>/overtAttention/set_mode</code> service and the <code>robotLocalization/pose</code> topic to be available, which can be hosted by running the <code>overtAttention</code> and <code>robotLocalization</code> node in the <code>cssr_system</code> package or running the <code>gestureExecutionTestStub</code> and <code>gestureExecutionTestDriver</code> in the <code>unit_tests</code> package before running the <code>gestureExecution</code>  node: </span>

         - <span style="color: #cccccc; font-weight: bold">(Option 1A):</span> Run the <code>overtAttention</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system overtAttention
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 1B):</span> Run the <code>robotLocalization</code> node of the <code>cssr_system</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2A):</span> Run the <code>gestureExecutionTestStub</code> of the <code>unit_tests</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun unit_tests gestureExecutionTestStub
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2B):</span> Run the <code>gestureExecutionTestDriver</code> of the <code>unit_tests</code> package (in a new terminal) passing the robot pose (robot_x, robot_y, robot_theta) as arguments
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun unit_tests gestureExecutionTestDriver <robot_x> <robot_y> <robot_theta>
            ```
            <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
               <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
               <span style="color: #cccccc;">Ensure that the robot pose <code>robot_x</code>, <code>robot_y</code> and <code>robot_theta</code> are correctly set based on your robot's position in the world. If these arguments are not supplied, the robot assumes its position as origin <code>(0, 0, 0)</code> </span>
            </div>
         </div>
       -->


## Executing Gestures
Upon launching the node, the hosted service (`/gestureExecution/perform_gesture`) is available and ready to be invoked. This can be verified by running the following command in a new terminal:

   ```bash
   rosservice list | grep /gestureExecution/perform_gesture
   ```

The command below invokes the service to execute a gesture (run in a new terminal) with the request parameters defined below:

   ```bash
   rosservice call /gestureExecution/perform_gesture -- <gesture_type> <gesture_id> <gesture_duration> <bow_nod_angle> <point_location_x> <point_location_y> <point_location_z>
   ```
### Service Request Parameters
#### 1. Gesture Types (gesture_type)
- `deictic`: Point at a location/object in the environment, specified by the 3D coordinates of the location/object.
- `iconic`: Communicate with/without speech, specified by the ID.
- `bow`: Bow in a respectful manner by tilting the robot's torso, specified by the degree of bowing.
- `nod`: Nod in agreement moving the robot's head up and down, specified by the degree of nodding.

#### 2. Gesture IDs (gesture_id)
- `01`: Welcome iconic gesture
- `02`: Welcome iconic gesture
- `03`: Goodbye iconic gesture
- `04`: Handshake iconic gesture
- `05`: Handshake iconic gesture

#### 3. Gesture Duration (gesture_duration)
This value defines the duration of the gesture (from start to end) and is specified in `milliseconds`. The minimum duration is `1000 ms` and the maximum duration is `10000 ms`.

#### 4. Bow/Nod Angle (bow_nod_angle)
This value is specified in degrees and indicates the degree to which the robot will bow (if bow gesture) or nod (if nod gesture). The minimum angle is `0 degrees` and the maximum angle is `45 degrees`

#### 5. Pointing Location (point_location_x, point_location_y, point_location_z)
These three values specify the location in the world (`in meters`) for the robot to point at in deictic gestures.

### Sample Invocations
- <span style="color: #cccccc; font-weight: bold;">Deictic (Pointing) gesture at (`3, 3, 0.82`) for `2000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- deictic 01 2000 0 3 3 0.82
   ```

* <span style="color: #cccccc; font-weight: bold;">Iconic open hands (`welcome`) gesture for `3000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- iconic 01 3000 0 0 0 0 
   ```
   <span style="color: #cccccc; font-weight: bold;">or</span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- iconic 02 3000 0 0 0 0 
   ```

- <span style="color: #cccccc; font-weight: bold;">Iconic wave (`goodbye`) gesture for `4000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- iconic 03 4000 0 0 0 0 
   ```

- <span style="color: #cccccc; font-weight: bold;">Iconic `handshake` gesture for `3000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- iconic 04 3000 0 0 0 0 
   ```
   <span style="color: #cccccc; font-weight: bold;">or</span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- iconic 05 3000 0 0 0 0 
   ```

- <span style="color: #cccccc; font-weight: bold;">Bowing at `30 degrees` for `2000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- bow 01 2000 30 0 0 0
   ```

- <span style="color: #cccccc; font-weight: bold;">Nodding at `15 degrees` for `1000 ms`: </span>
   ```sh
   rosservice call /gestureExecution/perform_gesture -- nod 01 1000 15 0 0 0 
   ```


## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, gesture types, debugging processes, and the overall functionality of the gestureExecution node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.1.pdf" style="color: #66b3ff;">D5.5.1 Gesture Execution</a>.These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:aakinade@andrew.cmu.edu">aakinade@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-01-07
