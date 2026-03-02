<div align="center">
  <h1>Robot Mission Interpreter</h1>
</div>


<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The ``Robot Mission Interpreter`` functions as a central ROS node within the CSSR4Africa software architecture, orchestrating
system behavior through ROS service calls and publish-subscribe mechanisms. It performs two key functions translating robot mission specifications into executable robot commands, and processing real-time sensor data and status updates from other ROS nodes to guide robot behavior.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.4.3 Robot Mission Interpreter](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf).



# Running the Robot Mission Interpreter (behaviorController) Node
## Steps
1. **Install the required software components:**
   - espeak
    
        eSpeak is a compact, open-source software speech synthesizer for English and other languages. It's used to convert text to spoken voice, which can be useful for debugging the execution of the robot mission execution.
        If you want the audio debugging functionality, then this software must be installed.
        
        To install eSpeak on a Debian-based Linux system (e.g., Ubuntu), run the following command in the terminal:
        ```bash
        sudo apt install espeak
        ```

    - BehaviorTree.CPP

        BehaviorTree.CPP is an open-source C++ library designed to implement, read, and execute behavior trees. The behaviorController node was built by importing the fundamental components from this library, including the core behavior tree types, the XML parsing functionality for loading tree specifications at runtime, and the tree execution engine. This dependency provides all the necessary building blocks for constructing and executing behavior trees while allowing the mission interpreter to focus on implementing the specific behaviors and actions.
        
        To install BehaviorTree.CPP on a Debian-based Linux system (e.g., Ubuntu), run the following command in the terminal:
        ```bash
        sudo apt-get install libzmq3-dev sqlite3 libsqlite3-dev libgtest-dev

        git clone --depth 1 --branch 4.6.2 https://github.com/BehaviorTree/BehaviorTree.CPP.git

        cd BehaviorTree.CPP

        cmake -DCMAKE_CXX_STANDARD=17 .

        make

        sudo make install
        ```
    - Pepper Robot Environment

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
   | `scenarioSpecification` | The robot mission specification file in XML format. File should be present in the `data` folder. | `labTour` |
   | `verboseMode` | Whether diagnostic data should be printed to the terminal. | `false` |
   | `asrEnabled` | Whether Automatic Speech Recognition is enabled on the platform or not | `true` |
   | `audioDebugMode` | Whether the debug audio should be on or not | `false` |   

4. **Run the `behaviorController` from the `cssr_system`  package:**
   
   Follow the steps below. Run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
         roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=true launch_actuators:=true 
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
    <span style="color: #cccccc;">Running the <code>behaviorController</code> node requires the following <span style="color: #ff3333; font-weight: bold;">services </span>:
          
        - /animateBehaviour/set_activation
        - /gestureExecution/perform_gesture
        - /overtAttention/set_mode
        - /robotNavigation/set_goal
        - /speechEvent/set_language
        - /tabletEvent/prompt_and_get_response"
        - /textToSpeech/say_text
    
    ... and the following <span style="color: #ff3333; font-weight: bold;">topics </span>:

        - /overtAttention/mode
        - /speechEvent/text

    During initialization, the `behaviorController` node checks for the availability of the required servers and topics in the order listed above. If any are unavailable, the node will terminate and display which server or topic was not found.

    To enable these servers and topics, there are two possible options.

<div>
<span>Actual ROS nodes:</span>

- <span style="color: #cccccc; font-weight: bold">1:</span> Run the <code>overtAttention</code> node of the <code>cssr_system</code> package (in a new terminal):
    ```sh
    cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system overtAttention
    ```
- <span style="color: #cccccc; font-weight: bold">2:</span> Run the <code>animateBehaviour</code> node of the <code>cssr_system</code> package (in a new terminal):
    ```sh
    cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system animateBehaviour
    ```
- <span style="color: #cccccc; font-weight: bold">3:</span> Run the <code>gestureExecution</code> node of the <code>cssr_system</code> package (in a new terminal):
    ```sh
    cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system gestureExecution
    ```

<!-- Verify and add the rest as they get accepted into the repo -->

<!-- - <span style="color: #cccccc; font-weight: bold">4:</span> Run the <code>robotNavigation</code> node of the <code>cssr_system</code> package (in a new terminal):
```sh
cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system robotNavigation
```
- <span style="color: #cccccc; font-weight: bold">5:</span> Run the <code>speechEvent</code> node of the <code>cssr_system</code> package (in a new terminal):
```sh
cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system speechEvent
```
- <span style="color: #cccccc; font-weight: bold">6:</span> Run the <code>tabletEvent</code> node of the <code>cssr_system</code> package (in a new terminal):
```sh
cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system tabletEvent
```
- <span style="color: #cccccc; font-weight: bold">7:</span> Run the <code>textToSpeech</code> node of the <code>cssr_system</code> package (in a new terminal):
```sh
    cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system textToSpeech
``` -->
</div>

<div>

<span>Drivers and Stubs from the unit tests</span>
  - <span style="color: #cccccc; font-weight: bold">1:</span> Run the <code>behaviorControllerStub</code> of the <code>unit_tests</code> package (in a new terminal):
    ```sh
    cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun unit_tests behaviorControllerTestStub
    ```
- <span style="color: #cccccc; font-weight: bold">2: </span> Run the <code>behaviorControllerDriver</code> of the <code>unit_tests</code> package (in a new terminal):
    ```sh
    cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun unit_tests behaviorControllerTestDriver
    ```
</div>


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