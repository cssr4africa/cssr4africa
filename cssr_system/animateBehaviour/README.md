<div align="center">
  <h1>Animate Behaviour Module for ROS</h1>
</div>


<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The animateBehaviour ROS node enhances robot interactions by creating lifelike presence through subtle, continuous movements. It achieves this by generating random patterns of body movements, hand flexing, and base rotation while keeping joint positions close to their default home values. The node controls all joints except headYaw and headPitch, and supports base rotation without forward movement. Movement ranges are configurable, with actual motions randomly sampled within specified limits. To respect culturally sensitive social interactions, these animations can be selectively enabled or disabled through a ROS service, allowing individual or combined activation of body, hand, and rotational movements. The system is designed to work seamlessly with both physical robots and simulators through platform-specific topic mapping configurations.

# Documentation
Accompnaying this code, there are deliverable reports that provides a detailed explanation of the code and how to run the tests. The deliverable reports are can be found in [D5.2 Animate Behaviour Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.2_Animate_Behaviour_Manual.pdf)


 
# Run the Animate Behaviour Node
## Physical Robot 
### Steps
1. Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

2. Clone and build:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_rob_ws/src
       ```
   - Clone the CSSR4Africa software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && catkin_make
       ```
       
3. **Update Configuration File**:
   Navigate to `~/workspace/pepper_rob_ws/src/cssr4africa/animateBehaviour/config/animateBehaviourConfiguration.ini` and update the configuration according to the key-value pairs below:

      | Parameter        | Description                | Values                    |
      |------------------|----------------------------|---------------------------|
      | platform         | Target platform            | robot                     |
      | behaviour        | Type of behaviour          | hands                     |
      | simulatorTopics  | Simulator topic file       | simulatorTopics.dat       |
      | robotTopics      | Robot topic file           | pepperTopics.dat          |
      | verboseMode      | Enable verbose mode        | true or false             |
      | rotMaximumRange  | Maximum rotation range     | 0.3                       |
      | selectedRange    | Selected rotation range    | 0.5                       |
      | armMaximumRange  | Maximum arm joint ranges   | 0.2, 0.2, 0.2, 0.35, 0.2  |
      | handMaximumRange | Maximum hand joint range   | 0.7                       |
      | legMaximumRange  | Maximum leg joint ranges   | 0.1, 0.1, 0.08            |
      | gestureDuration  | Duration of each gesture   | 1.0 seconds               |
      | numPoints        | Number of movement points  | 100                       |
      | numPointsLeg     | Number of leg points       | 2                         |
      | legRepeatFactor  | Leg movement repeat factor | 8                         |

   - To execute the animate behaviour on the physical platform, change the first line of `animateBehaviourConfiguration.ini` file in the config folder to “`platform robot`”. 
   - Set the behavior you want to test to one of the following options: `body`, `hands`, `rotation`, or `All`.

   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.2Animate_Behaviour_Manual.pdf" style="color: #66b3ff;">D5.2 Animate Behaviour Manual</a>. Otherwise, the preferred values are the ones already set in the configuration file.</span>
  </div>

4. Run the animate behaviour from `cssr_system`  package. Follow below steps.

    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws
          source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          cd .. && roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=172.29.111.240 network_interface:=wlp0s20f3
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP address <code>172.29.111.240</code> and the network interface <code>wlp0s20f3</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the animate behavior node.
         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
               <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
               <span style="color: #cccccc;">
                  Before running the node, ensure that the autonomous mode is disabled. If it is not, follow the steps below to disable it:
                  <ol style="color: #cccccc; margin-top: 10px;">
                        <li>SSH into the robot: <code>ssh nao@172.29.111.230</code></li>
                        <li>Disable autonomous mode: <code>qicli call ALAutonomousLife.setState disabled</code></li>
                        <li>Wake up the robot: <code>qicli call ALMotion.wakeUp</code></li>
                  </ol>
               </span>
            </div>
            
        ```bash
          cd $HOME/workspace/pepper_rob_ws
          cd .. && source devel/setup.bash
          cd .. && rosrun cssr_system animateBehaviour
        ```

    - To enable animate behavior, open a new terminal and run the code below.
        ```bash
          cd $HOME/workspace/pepper_rob_ws
          cd .. && source devel/setup.bash
          cd .. && rosservice call /animateBehaviour/setActivation "state: 'enabled'"
        ```
    - If you want to disable the animate behavior, run:
        ```bash
          cd .. && rosservice call /animateBehaviour/setActivation "state: 'disabled'"
        ```
## Simulator Robot

### Steps
1. **Install the required software components**:
   Set up the development environment for controlling the Pepper robot in the simulated environment. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

2. **Clone and Build**:
   - Move to the source directory of the workspace:
      ```bash
      cd $HOME/workspace/pepper_sim_ws/src
      ```
   - Clone the CSSR4Africa software from the GitHub repository:
      ```bash
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files:
      ```bash
      cd .. && catkin_make
      ```

3. **Update Configuration File**:
   Navigate to `~/workspace/pepper_sim_ws/src/cssr4africa/animateBehaviour/config/animateBehaviourConfiguration.ini` and update the configuration according to the key-value pairs below:

      | Parameter        | Description                | Values                    |
      |------------------|----------------------------|---------------------------|
      | platform         | Target platform            | simulator                 |
      | behaviour        | Type of behaviour          | hands                     |
      | simulatorTopics  | Simulator topic file       | simulatorTopics.dat       |
      | robotTopics      | Robot topic file           | pepperTopics.dat          |
      | verboseMode      | Enable verbose mode        | true or false             |
      | rotMaximumRange  | Maximum rotation range     | 0.3                       |
      | selectedRange    | Selected rotation range    | 0.5                       |
      | armMaximumRange  | Maximum arm joint ranges   | 0.2, 0.2, 0.2, 0.35, 0.2  |
      | handMaximumRange | Maximum hand joint range   | 0.7                       |
      | legMaximumRange  | Maximum leg joint ranges   | 0.1, 0.1, 0.08            |
      | gestureDuration  | Duration of each gesture   | 1.0 seconds               |
      | numPoints        | Number of movement points  | 100                       |
      | numPointsLeg     | Number of leg points       | 2                         |
      | legRepeatFactor  | Leg movement repeat factor | 8                         |

   - To execute the animate behaviour on the physical platform, change the first line of `animateBehaviourConfiguration.ini` file in the config folder to “`platform simulator`”. 
   - Set the behavior you want to test to one of the following options: `body`, `hands`, `rotation`, or `All`.

   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.2Animate_Behaviour_Manual.pdf" style="color: #66b3ff;">D5.2 Animate Behaviour Manual</a>. Otherwise, the preferred values are the ones already set in the configuration file.</span>
  </div>

4. **Run the Animate Behavior from `cssr_system` package**:
   - Source the workspace in the first terminal:
      ```bash
      cd $HOME/workspace/pepper_sim_ws
      source devel/setup.bash
      ```
   - Launch the simulator:
      ```bash
      roslaunch cssr_system cssrSystemLaunchSimulator.launch
      ```
   - Open a new terminal to launch the animate behavior node:
      ```bash
      cd $HOME/workspace/pepper_sim_ws
      source devel/setup.bash
      roslaunch cssr_system animateBehaviour
      ```
   - To enable animate behavior, open a new terminal and run the code below:
      ```bash
      cd $HOME/workspace/pepper_sim_ws
      source devel/setup.bash
      rosservice call /animateBehaviour/set_activation "state: 'enabled'"
      ```
   - If you want to disable the animate behavior, run:
      ```bash
      rosservice call /animateBehaviour/set_activation "state: 'disabled'"
      ```
## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, animate behavior types, debugging processes, and the overall functionality of the animate behavior node, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/D5.2_Animate_Behaviour_Manual.pdf" style="color: #66b3ff;">D5.2 Animate Behaviour Manual</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ebirhan@andrew.cmu.edu">ebirhan@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  


Date:  2025-01-10
