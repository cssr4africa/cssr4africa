<div align="center">
  <h1>Robot Mission Interpreter</h1>
</div>


<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The ``Robot Mission Interpreter`` functions as a central ROS node within the CSSR4Africa software architecture, orchestrating
system behavior through ROS service calls and publish-subscribe mechanisms. It performs two key functions translating robot mission specifications into executable robot commands, and processing real-time sensor data and status updates from other ROS nodes to guide robot behavior.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.4.3 Robot Mission Interpreter](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf).



# Running the Robot Mission Interpreter (behaviorController) Node
## Steps
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
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf" style="color: #66b3ff;">D5.4.3 Robot Mission Interpreter</a>. The default values are the ones already set in the configuration file and specified below.</span>
   </div>

   Navigate to the configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/behaviorController/config/behaviorControllerConfiguration.ini` and update the configuration according to the key-value pairs below. The default values are the ones already present. If a different behavior is desired, the values should be changed accordingly.

   | Parameter | Description | Default Value |
   |-----------|-------------|---------|
   | `scenarioSpecification` | The robot mission specification file in XML format. File should be present in the `data` folder. | `lab_tour` |
   | `verboseMode` | Whether diagnostic data should be printed to the terminal. | `true` |
   | `asrEnabled` | Whether Automatic Speech Recognition is enabled on the platform or not | `false` |
   | `testMode` | Whether the test sequence should run or not | `false` |   

4. **Run the `behaviorController` from the `cssr_system`  package:**
   
   Follow the steps below. Run in different terminals.
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
    - Open a new terminal to launch the `behaviorController` node.
        ```bash
        cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system behaviorController
        ```
        
    
    
    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
    <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
    <span style="color: #cccccc;">Running the <code>behaviorController</code> node requires the following <span style="color: #ff3333; font-weight: bold;">servers </span>:
          
        - /animateBehaviour/set_activation
        - /gestureExecution/perform_gesture
        - /overtAttention/set_mode
        - /robotNavigation/set_goal
        - /speechEvent/set_language
        - /tabletEvent/prompt_and_get_response"
        - /textToSpeech/say_text
          

    
    ... and the following <span style="color: #ff3333; font-weight: bold;">topics </span>:

        - /faceDetection/data
        - /overtAttention/mode
        - /speechEvent/text

    During initialization, the `behaviorController` node checks for the availability of the required servers and topics in the order listed above. If any are unavailable, the node will terminate and display which server or topic was not found.

    Note that this check is performed only once at startup. After initialization, control is handed over to the mission execution logic, and subsequent behavior depends on the specific robot mission specification that has been loaded.
         
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ttefferi@andrew.cmu.edu">ttefferi@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-04-08