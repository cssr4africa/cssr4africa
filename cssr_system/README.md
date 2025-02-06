### The CSSR4Africa Project -- CSSR System

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:90%; height:auto;">

The is directory contains the code and resources for the CSSR4Africa project, based on the system architecture which is available in the [CSSR4Africa System Architecture](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.1.pdf). As of now, the available nodes are [animateBehaviour](/animateBehaviour/), [gestureExecution](/gestureExecution/), and [overtAttention](\overtAttention/). These software have been passed through the software integration process as reported in the [System Integration and Quality Assurance](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.5.pdf). This integration process is based on the standards set out in the [Software Engineering Standards Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.2.pdf) and the [System Integration and Quality Assurance Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.4.pdf). 



# Documentation
Accompanying this code will be the user manual that provides guide on how to set up the `cssr_system`. The user manual will be available on the [cssr4africa](https://cssr4africa.github.io/) website.


 
# Running the cssr system
## Physical Robot 
### Steps
1. **Install the required software components:**
   
   If not already done, set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

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
      cd $HOME/workspace/pepper_rob_ws && catkin_make && source devel/setup.bash 
       ```

4. **Run the `cssr_system` mission from the `cssr_system`  package:**
   
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
    - Open a new terminal to launch the `cssr_system` mission.
        ```bash
         cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system cssrSystemLaunchMission.launch launch_controller:=true
        ```

          <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Setting the argument <code>launch_controller</code> to <code>true</code> runs the mission with the <code>behaviourController</code> based on its configuration, while setting it to <false> only launches all the nodes (if available) without launching te mission. If this argument is not provided, a default value of <code>false</code> is used.</span>
        </div>


## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:david@vernon.eu">david@vernon.eu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-02-07
