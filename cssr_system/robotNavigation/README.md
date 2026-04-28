<div align="center">
  <h1>Robot Navigation</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotNavigation` ROS node enables autonomous navigation capabilities for the Pepper humanoid robot. This node provides path planning and navigation functionalities, allowing the robot to navigate from its current position to a specified goal location while avoiding obstacles in the environment.

The package implements various path planning algorithms including A*, Dijkstra, BFS, and DFS and can be configured to respect social distancing norms during navigation. When social distance mode is enabled, the passing distance value is read from the `cultureKnowledgeBaseInput.dat` file in the `behaviorController` package to enforce culturally appropriate social distancing during navigation.

The node supports two navigation modes: **CAD** (using pre-built maps with internal path planning) and **SLAM** (using the ROS navigation stack with `move_base` for real-time localization and obstacle avoidance). When running in SLAM mode, the node automatically launches the required navigation stack components (map_server, AMCL, move_base, and RViz) as a subprocess.

The node provides both a **service-based interface** (blocking, for simple call-and-wait usage) and an **action-based interface** (non-blocking, with real-time feedback and goal preemption/cancellation support). Both interfaces are available simultaneously when the node is running.

To accommodate diverse navigation scenarios, parameters such as path planning algorithm, social distance mode, navigation mode, and verbose output are configurable. This package is designed for use with physical Pepper robots, allowing seamless integration into larger robotics applications through ROS topic, service, and action interfaces.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.5.4 Robot Navigation](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.4.pdf).

# Dependencies

The `robotNavigation` node is part of the `cssr_system` package. The following dependencies must be declared in the `cssr_system` package-level files for the node to compile and run correctly.

### Build Dependencies (CMakeLists.txt)

The following entries must be present in the `find_package(catkin REQUIRED COMPONENTS ...)` block of `cssr_system/CMakeLists.txt`:

```cmake
actionlib
actionlib_msgs
move_base_msgs
```

The following entries must be present in the `generate_messages(DEPENDENCIES ...)` block:

```cmake
actionlib_msgs
```

The following action file generation block must be present:

```cmake
add_action_files(
  DIRECTORY
  robotNavigation/action
  FILES
  setGoal.action
  setPose.action
)
```

### Runtime Dependencies (package.xml)

The following entries must be present in `cssr_system/package.xml` for the node to compile:

```xml
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>move_base_msgs</build_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
<exec_depend>move_base_msgs</exec_depend>
```

The following entries must be present in `cssr_system/package.xml` for the SLAM navigation stack to run at runtime:

```xml
<exec_depend>map_server</exec_depend>
<exec_depend>amcl</exec_depend>
<exec_depend>move_base</exec_depend>
<exec_depend>teb_local_planner</exec_depend>
<exec_depend>global_planner</exec_depend>
<exec_depend>rviz</exec_depend>
```

### Installing Dependencies

The following ROS packages are required but are **not** included in `ros-noetic-desktop-full`. They must be installed explicitly:

```bash
sudo apt-get install ros-noetic-navigation ros-noetic-teb-local-planner
```

- `ros-noetic-navigation` installs: `map-server`, `amcl`, `move-base`, `move-base-msgs`, `global-planner`, and related navigation packages.
- `ros-noetic-teb-local-planner` installs: the TEB local planner used by `move_base` for local trajectory planning.

After installing the above, run `rosdep` from the workspace root to verify all remaining dependencies are satisfied:

```bash
cd $HOME/workspace/pepper_rob_ws
rosdep install --from-paths src --ignore-src -r -y
```

Then build the workspace:

```bash
catkin_make
source devel/setup.bash
```

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
   - Install dependencies
      ```bash
      cd ..
      rosdep install --from-paths src --ignore-src -r -y
      ```
   - Build the source files
      ```bash
      catkin_make
      source devel/setup.bash
      ```

3. **Update Configuration File:**

   Navigate to the configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/robotNavigation/config/robotNavigationConfiguration.ini` and update the configuration according to your needs:

   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `environmentMap` | Environmental map file | `environmentMap.png` | `environmentMap.png` |
   | `configurationMap` | Configuration map file | `configurationSpaceMap.png` | `configurationSpaceMap.png` |
   | `pathPlanning` | Path planning algorithm (CAD mode) | `astar`, `dijkstra`, `bfs`, `dfs` | `astar` |
   | `socialDistance` | Social distance mode. When enabled, reads `passingDistance` from `cultureKnowledgeBaseInput.dat` | `true`, `false` | `true` |
   | `robotTopics` | Physical robot topic mapping file | `pepperTopics.dat` | `pepperTopics.dat` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` | `false` |
   | `robotType` | Choose between the new and old pepper robot | `new`, `old` <br>`(Old:172.29.111.230)` `(new:172.29.111.240)` | `old` |
   | `navigationMode` | Navigation mode | `CAD`, `SLAM` | `CAD` |


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

         If the complete softwares are integrated and we have a system wide launch file, you can use the following command to connect to the robot.

         ```bash
         roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> launch_sensors:=true launch_actuators:=true
         ```

    - Open a new terminal to start the `robotLocalization` node:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun cssr_system robotLocalization
        ```

    - Open a new terminal to launch the `robotNavigation` node:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun cssr_system robotNavigation
        ```

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">Running the <code>robotNavigation</code> node requires the <code>/robotLocalization/pose</code> topic to be available, which can be hosted by running the <code>robotLocalization (Recommended)</code> node in the <code>cssr_system</code> package or running the <code>robotNavigationDriver</code> in the <code>unit_tests</code> package to get the current position and orientation of the robot for <code>robotNavigation</code> node functionality. </span>
         </div>

    - Set the initial robot pose before sending a goal:
        ```bash
        rosservice call /robotLocalization/set_pose 2.0 7.8 270.0
        ```

### Prerequisites for SLAM Mode

   When `navigationMode` is set to `SLAM`, the node automatically launches the navigation stack (map_server, AMCL, move_base, and RViz) as a subprocess. The following prerequisites must be met before running in SLAM mode:

   1. The `/scan` topic must be available. This topic provides laser scan data required by AMCL and the costmap layers. The hardware setup may vary, but the topic name must be `/scan`. For the current laboratory setup, the LiDAR is a YDLidar G2 connected to a Jetson Orin Nano:
      ```bash
      ssh roboticslab@<jetson_ip>
      roslaunch ydlidar_ros_driver G2.launch
      ```

   2. The robot driver must be running and publishing odometry on `/naoqi_driver/odom`.

   3. The SLAM navigation stack configuration files are located in `robotNavigation/slam/`. The map files used by the map server are in `robotNavigation/slam/maps/`.

5. **Using the `robotNavigation` Interfaces:**

   The node provides two interfaces for sending navigation goals: a **service interface** (blocking) and an **action interface** (non-blocking with feedback). Both are available simultaneously.

   ### Subscribed Topics

   | Topic | Type | Description |
   |-------|------|-------------|
   | `/robotLocalization/pose` | `geometry_msgs/Pose2D` | Current robot pose (mandatory) |

   ### Published Topics

   | Topic | Type | Mode | Description |
   |-------|------|------|-------------|
   | `[wheels_topic]` | `geometry_msgs/Twist` | CAD | Velocity commands |
   | `/joint_angles` | `naoqi_bridge_msgs/JointAnglesWithSpeed` | Both | Joint commands |
   | `move_base_simple/goal` | `geometry_msgs/PoseStamped` | SLAM | Goal for move_base |
   | `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | SLAM | Initial pose for AMCL |

   ### Service Interface (Blocking)

   | Service | Type | Description |
   |---------|------|-------------|
   | `/robotNavigation/set_goal` | `cssr_system/setGoal` | Send a navigation goal (blocking) |
   | `/robotNavigation/set_pose` | `cssr_system/setGoal` | Set robot pose (SLAM mode) |

   **Service Request Fields:**
   - `goal_x` (float64): X-coordinate of the goal location in meters
   - `goal_y` (float64): Y-coordinate of the goal location in meters
   - `goal_theta` (float64): Final orientation at the goal in degrees

   **Service Response Fields:**
   - `navigation_goal_success` (uint8): 1 = success, 0 = failure

   Verify the service is available:
   ```bash
   rosservice list | grep /robotNavigation/set_goal
   ```

   **Send a Goal via Service:**
   ```bash
   rosservice call /robotNavigation/set_goal 2.0 6.0 270.0
   ```

   If a **negative** value of theta needs to be passed, use one of the following formats (the `-` at the beginning of an argument is normally interpreted as a flag in Unix/Linux):
   ```bash
   rosservice call /robotNavigation/set_goal -- 2.0 7.8 -45.0
   ```
   ```bash
   rosservice call /robotNavigation/set_goal "{goal_x: 2.0, goal_y: 7.8, goal_theta: -45.0}"
   ```

   ### Action Interface (Non-Blocking with Feedback)

   | Action | Type | Description |
   |--------|------|-------------|
   | `/robotNavigation/set_goal` | `cssr_system/setGoalAction` | Send a navigation goal with feedback and preemption |
   | `/robotNavigation/set_pose` | `cssr_system/setPoseAction` | Set robot pose with status feedback |

   **setGoal Action Definition:**

   | Field | Type | Description |
   |-------|------|-------------|
   | **Goal** | | |
   | `goal_x` | float64 | X-coordinate in meters |
   | `goal_y` | float64 | Y-coordinate in meters |
   | `goal_theta` | float64 | Orientation in degrees |
   | **Result** | | |
   | `navigation_goal_success` | uint8 | 1 = success, 0 = failure |
   | `final_x` | float64 | Final X position |
   | `final_y` | float64 | Final Y position |
   | `final_theta` | float64 | Final orientation in degrees |
   | **Feedback** | | |
   | `distance_remaining` | float64 | Distance to goal in meters |
   | `current_x` | float64 | Current X position |
   | `current_y` | float64 | Current Y position |
   | `current_theta` | float64 | Current orientation in degrees |

   **setPose Action Definition:**

   | Field | Type | Description |
   |-------|------|-------------|
   | **Goal** | | |
   | `pose_x` | float64 | X-coordinate in meters |
   | `pose_y` | float64 | Y-coordinate in meters |
   | `pose_theta` | float64 | Orientation in degrees |
   | **Result** | | |
   | `pose_set_success` | uint8 | 1 = success |
   | **Feedback** | | |
   | `status` | string | Status description |

   **Send a Goal via Action:**
   ```bash
   rostopic pub -1 /robotNavigation/set_goal/goal cssr_system/setGoalActionGoal "goal: {goal_x: 2.0, goal_y: 6.0, goal_theta: 270.0}"
   ```

   **Monitor Action Feedback:**
   ```bash
   rostopic echo /robotNavigation/set_goal/feedback
   ```

   **Monitor Action Result:**
   ```bash
   rostopic echo /robotNavigation/set_goal/result
   ```

   **Cancel an Active Goal:**
   ```bash
   rostopic pub -1 /robotNavigation/set_goal/cancel actionlib_msgs/GoalID "{}"
   ```

   ### Setting Robot Pose

   **Via Service:**
   ```bash
   rosservice call /robotNavigation/set_pose 2.0 7.8 270.0
   ```

   **Via Action:**
   ```bash
   rostopic pub -1 /robotNavigation/set_pose/goal cssr_system/setPoseActionGoal "goal: {pose_x: 2.0, pose_y: 7.8, pose_theta: 270.0}"
   ```

   You can also set the initial robot pose via the robot localization node:
   ```bash
   rosservice call /robotLocalization/set_pose 2.0 7.8 270.0
   ```

   **Echo the Pose Topic:**

   View the pose data assuming robot localization is running and publishing the current pose:
   ```bash
   rostopic echo /robotLocalization/pose
   ```

## Sample Usage

Below is a sample session demonstrating both service and action interfaces in CAD mode:

```bash
# Set the initial robot pose
rosservice call /robotLocalization/set_pose 2.0 7.8 270.0

# Send goal via service (blocking)
rosservice call /robotNavigation/set_goal 2.0 6.0 270
# navigation_goal_success: 1

# Send goal via action (non-blocking with feedback)
rostopic pub -1 /robotNavigation/set_goal/goal cssr_system/setGoalActionGoal "goal: {goal_x: 2.0, goal_y: 7.8, goal_theta: 270.0}"

# Send goal via action
rostopic pub -1 /robotNavigation/set_goal/goal cssr_system/setGoalActionGoal "goal: {goal_x: 2.0, goal_y: 6.0, goal_theta: 270.0}"
```

Below is a sample session demonstrating SLAM mode:

```bash
# Set the initial robot pose for SLAM
rosservice call /robotNavigation/set_pose 2.0 7.8 270.0

# Send goal via service (blocking)
rosservice call /robotNavigation/set_goal 2.0 6.0 270
# navigation_goal_success: 1

# Return to starting position
rosservice call /robotNavigation/set_goal 2.0 7.8 270
# navigation_goal_success: 1
```

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:bgirmash@andrew.cmu.edu">bgirmash@andrew.cmu.edu</a>, <a href="mailto:bgirmash@alumni.cmu.edu">bgirmash@alumni.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License
Funded by African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme

Date:   2026-04-05
