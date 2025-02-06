<div align="center">
  <h1>Overt Attention</h1>
</div>


<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `overtAttention` ROS node enhances the Pepper humanoid robot's ability to perform meaningful and dynamic attention behaviors, improving its interactive capabilities. This ROS node hosts a service that allows users to invoke a variety of attention behaviors, including looking at faces, scanning the environment, and focusing on a particular location.
The package leverages biological motion profiles to ensure natural and fluid execution, making interactions more engaging and lifelike. Attention behaviors are executed using the robot's built-in kinematic capabilities, and the system ensures that the motions are aligned with the robot's physical constraints.
This package is designed for use with thr physical Pepper robots, allowing seamless integration into larger robotics applications through ROS topic and service interfaces.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.3 Overt Attention](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.3.pdf).


 
# Run the Overt Attention Node
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
   
   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.3.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
   </div>

   Navigate to the configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/gestureExecution/config/gestureExecutionConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `camera` | Sensing device | `FrontCamera`, `RealSenseCamera` |
   | `realignmentThreshold` | Threshold on the angular difference between head and base that must be met before the head and base are re-aligned | `degree` |
   | `xOffsetToHeadYaw` | calibration constant that defines the conversion of the offset in the (horizontal) x-axis of an image | `degree` |
   | `yOffsetToHeadPitch` | calibration constant that defines the conversion of the offset in the (vertical) y-axis of an image | `degree` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` |
   | `simulatorTopics` | Physical robot topic mapping file | `simulatorTopics.dat` |
   | `socialAttentionMode` | Simulator robot topic mapping file | `saliency`, `random` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` |

   <!-- - To execute the overtAttention on the physical platform, change the first line of `overtAttentionConfiguration.ini` file in the config folder to “`platform robot`”.  -->
   - Change the second line of `overtAttentionConfiguration.ini` file in the config folder, "`camera`" parameter, to the desired camera sensor.
   - Modify the "`realignmentThreshold`" parameter on the third line of `overtAttentionConfiguration.ini` file to the angle where the head and the body need to be re-aligned.
   - Set the `xOffsetToHeadYaw` parameter on the fourth line of `overtAttentionConfiguration.ini` file to offset for the horizontal distance between the camera sensor and the "eyes" of the pepper robot.
   - Set the `yOffsetToHeadPitch` parameter on the fifth line of `overtAttentionConfiguration.ini` file to offset for the vertical distance between the camera sensor and the "eyes" of the pepper robot.
   - All actuators have a topic, through which actiuation is carried out. These topics are specified in a key-value pair format in a file which is defined by the "`robotTopics`" parameter and "`simulatorTopics`" parameter in the `overtAttentionConfiguration.ini` file for the physical and simulator robot respectively. These files are found in the data folder. The files are specified as "`simulatorTopics simulatorTopics.dat`" and "`pepperTopics pepperTopics.dat`".	
   - Change the `socialAttentionMode` parameter in the `overtAttentionConfiguration.ini` file to to the desired method for social attention.
   - The system is capable of printing diagnostic informqation to the terminal. This behaviour is controlled by the last key-value pair in the `overtAttentionConfiguration.ini` file, "`verboseMode`" parameter key.

4. **Run the `overtAttention` from the `cssr_system`  package:**
   
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
    - Open a new terminal to launch the `overtAttention` node.
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>overtAttention</code> node requires the <code>/faceDetection/data</code>, <code>/soundDetection/direction</code>, and the <code>robotLocalization/pose</code> topics to be available, which can be hosted by running the <code>faceDetection</code>, <code>soundDetection</code>, and <code>robotLocalization</code> node in the <code>cssr_system</code> package or running the <code>overtAttentionTestDriver</code> in the <code>unit_tests</code> package before running the <code>overtAttention</code>  node: </span>

         - <span style="color: #cccccc; font-weight: bold">(Option 1A):</span>  Run the <code>faceDetection</code> node of the <code>cssr_system</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system face_detetection_application.py
            ``` 
         - <span style="color: #cccccc; font-weight: bold">(Option 1B):</span> Run the <code>soundDetection</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system sound_detection_application.py
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 1C):</span> Run the <code>robotLocalization</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2): </span> Run the <code>overtAttentionTestDriver</code> of the <code>unit_tests</code> package (in a new terminal) passing the robot pose (robot_x, robot_y, robot_theta) as arguments:
            ```sh
            cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun unit_tests overtAttentionTestDriver <robot_x> <robot_y> <robot_theta>
            ```
            <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
               <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
               <span style="color: #cccccc;">Ensure that the robot pose <code>robot_x</code>, <code>robot_y</code> and <code>robot_theta</code> are correctly set based on your robot's position in the world. If these arguments are not supplied, the robot assumes its position as origin <code>(0, 0, 0)</code> </span>
            </div>
         </div>
         
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system overtAttention
        ```
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
   
   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.3.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
   </div>

   Navigate to the configuration file located at `$HOME/workspace/pepper_sim_ws/src/cssr4africa/gestureExecution/config/gestureExecutionConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `camera` | Sensing device | `FrontCamera`, `RealSenseCamera` |
   | `realignmentThreshold` | Threshold on the angular difference between head and base that must be met before the head and base are re-aligned | `degree` |
   | `xOffsetToHeadYaw` | calibration constant that defines the conversion of the offset in the (horizontal) x-axis of an image | `degree` |
   | `yOffsetToHeadPitch` | calibration constant that defines the conversion of the offset in the (vertical) y-axis of an image | `degree` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` |
   | `simulatorTopics` | Physical robot topic mapping file | `simulatorTopics.dat` |
   | `socialAttentionMode` | Simulator robot topic mapping file | `saliency`, `random` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` |

   - To execute the overtAttention on the physical platform, change the first line of `overtAttentionConfiguration.ini` file in the config folder to “`platform robot`”. 
   - Change the second line of `overtAttentionConfiguration.ini` file in the config folder, "`camera`" parameter, to the desired camera sensor.
   - Modify the "`realignmentThreshold`" parameter on the third line of `overtAttentionConfiguration.ini` file to the angle where the head and the body need to be re-aligned.
   - Set the `xOffsetToHeadYaw` parameter on the fourth line of `overtAttentionConfiguration.ini` file to offset for the horizontal distance between the camera sensor and the "eyes" of the pepper robot.
   - Set the `yOffsetToHeadPitch` parameter on the fifth line of `overtAttentionConfiguration.ini` file to offset for the vertical distance between the camera sensor and the "eyes" of the pepper robot.
   - All actuators have a topic, through which actiuation is carried out. These topics are specified in a key-value pair format in a file which is defined by the "`robotTopics`" parameter and "`simulatorTopics`" parameter in the `overtAttentionConfiguration.ini` file for the physical and simulator robot respectively. These files are found in the data folder. The files are specified as "`simulatorTopics simulatorTopics.dat`" and "`pepperTopics pepperTopics.dat`".	
   - Change the `socialAttentionMode` parameter in the `overtAttentionConfiguration.ini` file to to the desired method for social attention.
   - The system is capable of printing diagnostic informqation to the terminal. This behaviour is controlled by the last key-value pair in the `overtAttentionConfiguration.ini` file, "`verboseMode`" parameter key.


4. **Run the `overtAttention` from the `cssr_system` package:**:
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
         cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the simulator robot:
        ```bash
         roslaunch cssr_system cssrSystemLaunchSimulator.launch
        ```
    - Open a new terminal to launch the `overtAttention` node.
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>overtAttention</code> node requires the <code>/faceDetection/data</code>, <code>/soundDetection/direction</code>, and the <code>robotLocalization/pose</code> topics to be available, which can be hosted by running the <code>faceDetection</code>, <code>soundDetection</code>, and <code>robotLocalization</code> node in the <code>cssr_system</code> package or running the <code>overtAttentionTestDriver</code> in the <code>unit_tests</code> package before running the <code>overtAttention</code>  node: </span>

         - <span style="color: #cccccc; font-weight: bold">(Option 1A):</span>  Run the <code>faceDetection</code> node of the <code>cssr_system</code> package (in a new terminal):
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system face_detetection_application.py
            ``` 
         - <span style="color: #cccccc; font-weight: bold">(Option 1B):</span> Run the <code>soundDetection</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system sound_detection_application.py
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 1C):</span> Run the <code>robotLocalization</code> node of the <code>cssr_system</code> package (in a new terminal): 
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
            ```
         - <span style="color: #cccccc; font-weight: bold">(Option 2): </span> Run the <code>overtAttentionTestDriver</code> of the <code>unit_tests</code> package (in a new terminal) passing the robot pose (robot_x, robot_y, robot_theta) as arguments:
            ```sh
            cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun unit_tests overtAttentionTestDriver <robot_x> <robot_y> <robot_theta>
            ```
            <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
               <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
               <span style="color: #cccccc;">Ensure that the robot pose <code>robot_x</code>, <code>robot_y</code> and <code>robot_theta</code> are correctly set based on your robot's position in the world. If these arguments are not supplied, the robot assumes its position as origin <code>(0, 0, 0)</code> </span>
            </div>
         </div>

        ```bash
         cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system overtAttention
        ```
         -->


## Executing Attention
Upon launching the node, the hosted service (`/overtAttention/set_mode`) is available and ready to be invoked. This can be verified by running the following command in a new terminal:

   ```sh
   rostopic list | grep /overtAttention/set_mode
   ```

The command below invokes the service to execute a gesture (run in a new terminal) with the request parameters defined below:

   ```sh
   rosservice call /overtAttention/set_mode -- "state: <mode> location_x: <x_coordinate> location_y: <y_coordinate> location_z: <z_coordinate> 
   ```
   
### Service Request Parameters
#### 1. State (mode)
- `scanning`: Scan the environment and look at interesting things.
- `location`: Look at a particular location.
- `social`: Look for faces and voices.
- `seeking`: Seek for mutual gaze.
- `disabled`: Disable the node.

#### 2. Attention Location (in meters)
- `location_x`: x coordinate of real world location to look at
- `location_y`: y coordinate of real world location to look at
- `location_z`: z coordinate of real world location to look at

### Sample Invocations
- <span style="color: #cccccc; font-weight: bold;">Location mode at (`3, 3, 0.82`): </span>
```sh
rosservice call /overtAttention/set_mode "state: 'location'  location_x: 3 location_y: 3 location_z: 0.82"
```
- <span style="color: #cccccc; font-weight: bold;">Scanning mode: </span>
```sh
rosservice call /overtAttention/set_mode "state: 'scanning'  location_x: 0 location_y: 0 location_z: 0"
```
- <span style="color: #cccccc; font-weight: bold;">Social mode: </span>
```sh
rosservice call /overtAttention/set_mode "state: 'social'  location_x: 0 location_y: 0 location_z: 0"
```
- <span style="color: #cccccc; font-weight: bold;">Seeking mode: </span>
```sh
rosservice call /overtAttention/set_mode "state: 'seeking'  location_x: 0 location_y: 0 location_z: 0"
```
- <span style="color: #cccccc; font-weight: bold;">Disabled mode: </span>
```sh
rosservice call /overtAttention/set_mode "state: 'disabled'  location_x: 0 location_y: 0 location_z: 0"
```

## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, attention modes, debugging processes, and the overall functionality of the overtAttention node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.3.pdf" style="color: #66b3ff;">D5.3 Overt Attention</a>.These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:mdanso@andrew.cmu.edu">mdanso@andrew.cmu.edu</a><br>, <a href="mailto:aakinade@andrew.cmu.edu">aakinade@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-01-10
