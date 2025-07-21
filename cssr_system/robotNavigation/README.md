<div align="center">
  <h1>Robot Navigation</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotNavigation` ROS node enables autonomous navigation capabilities for the Pepper humanoid robot. This node provides path planning and navigation functionalities, allowing the robot to navigate from its current position to a specified goal location while avoiding obstacles in the environment.

The package implements various path planning algorithms including A*, Dijkstra,BFS, and DFS and can be configured to respect social distancing norms during navigation. The system leverages environmental maps and configuration maps to plan optimal paths and executes smooth motion commands for the robot.

To accommodate diverse navigation scenarios, parameters such as path planning algorithm, social distance mode, and verbose output are configurable. This package is designed for use with physical Pepper robots, allowing seamless integration into larger robotics applications through ROS topic and service interfaces.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.5.4 Robot Navigation](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.4.pdf).

# Run the Robot Navigation Node


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
      cd ..
      catkin_make
      source devel/setup.bash 
      ```

3. **Update Configuration File:**
   
   Navigate to the configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/robotNavigation/config/robotNavigationConfiguration.ini` and update the configuration according to your needs:

   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `environmentMap` | Environmental map file | `environmentMap.png` | `environmentMap.png` |
   | `configurationMap` | Configuration map file | `configurationSpaceMap.png` | `configurationSpaceMap.png` |
   | `pathPlanning` | Path planning algorithm | `astar`, `dijkstra`, `bfs`, `dfs` | `astar` |
   | `socialDistance` | Social distance mode | `true`, `false` | `true` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` | `pepperTopics.dat` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` | `false` |
   | `robotType` | Choose between the new and old pepper robot | `new`, `old` <br>`(Old:172.111.29.230)` `(new:172.111.29.240)` | `new` |


4. **Run the `robotNavigation` from the `cssr_system` package:**
   
   Follow these steps, running in different terminals:
    -  Source the workspace in first terminal:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        ```
    -  ***Launch and connect to the robot*** (make sure the robot is connected to the network and the robot is powered on, and the robot has to be on the same network with the computer being used to connect to the robot):

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
            <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
            <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set in the unit test launch file based on your robot's configuration and your computer's network interface. </span>
         </div>

         ```bash
         roslaunch unit_tests robotNavigationTestLaunchRobot.launch 
        ```

        You can select the robot to be used by using the `robot_ip` parameter when running the above command. Other optional parameters that can be used are: `robot_port`, `roscore_ip`, and `network_interface`. An example is shown below.

        ```bash
         roslaunch unit_tests robotNavigationTestLaunchRobot.launch robot_ip:=172.29.111.240
        ```

         If the complete softwares are integrated and we have a system wide launch filem, you can us the following command to connect to the robot.

         ```bash
         roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=true launch_actuators:=true 
         ```

    - Open a new terminal to launch the `robotNavigation` node:
        ```bash
        cd $HOME/workspace/pepper_rob_ws 
        source devel/setup.bash 
        ```
    - To run the `robotNavigation` node, use the following command:
        ```bash
        rosrun cssr_system robotNavigation
        ```

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>robotNavigation</code> node requires the <code>/robotLocalization/pose</code> topic to be available, which can be hosted by running the <code>robotLocalization (Recommended)</code> node in the <code>cssr_system</code> package or running the <code>robotNavigationDriver</code> in the <code>unit_tests</code> package to get the current position and orientation of the robot for <code>robotNavigation</code> node functionality. </span>
         </div>

5. **Using the `robotNavigation` Services:**
  Upon launching the node, the hosted service (`/robotNavigation/set_goal`) is available and ready to be invoked. This can be verified by running the following command in a new terminal:

     ```bash
      rosservice list | grep /robotNavigation/set_goal
      ```
   **Echo the Pose Topic**
   View the pose data assuming robot localization is running and publishing the current pose:
   ```bash
   rostopic echo /robotNavigation/pose
   ```
    **Set the initial robot pose before sending a goal to navigation node**
    Use the set Pose Service to set the robot's pose:
    ```bash
    rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
    ```
    **Send a Goal**
    Send a goal to the robot:
    - Goal Location (<code>goal_x</code>, <code>goal_y</code>)
        - <code>goal_x</code>: X-coordinate of the goal location in meters
        - <code>goal_y</code>: Y-coordinate of the goal location in meters
    - Goal Orientation (<code>goal_theta</code>)    
        - <code>goal_theta</code>: Final orientation of the robot at the goal location in degrees
    
    - Sample Invocations (Navigate to position (2.0, 6.6) with 0-degree orientation:
    ```bash
    rosservice call /robotNavigation/set_goal 2.0 6.6 0.0
    ```
   - If **negative** value of theta needs to be passed as a goal, you have to use the following format. It is because of how the rosservice call command parses command-line arguments. This is standard behavior in Unix/Linux command-line tools, where <code> - </code> at the beginning of an argument is normally interpreted as a flag. 
   ```bash
   rosservice call /robotNavigation/set_goal -- 2.0 7.8 -45.0
   ```
   ```bash
   rosservice call /robotNavigation/set_goal "{goal_x: 2.0, goal_y: 7.8, goal_theta: -45.0}"
   ```
    **Setting Robot Pose**
    If you need to set the robot's pose (usually for initialization or testing), use the following service::
    ```bash
    rosservice call /robotLocalization/set_pose <x> <y> <theta>
    ```
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:bgirmash@andrew.cmu.edu">bgirmash@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-06-05
