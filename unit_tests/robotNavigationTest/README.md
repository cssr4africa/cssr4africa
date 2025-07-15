<div align="center">
  <h1>Robot Navigation Test</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotNavigationTest` ROS node provides a unit testing capabilities for the CSSR4Africa robot navigation system. This test suite validates path planning algorithms (BFS, Dijkstra, A*), navigation services, and boundary conditions using real robot hardware and map data.

The package implements testing of core navigation functionalities, including algorithm performance comparison, service integration validation, and boundary condition handling. The system leverages the actual navigation algorithms and map files to ensure validation of the navigation node.

To accommodate diverse testing scenarios, parameters such as individual test categories, algorithm selection, and output verbosity are configurable. This package is designed for use with physical Pepper robots and integrates with the existing CSSR4Africa navigation infrastructure.

# Documentation
Accompanying this code is comprehensive test documentation that provides detailed explanation of the test architecture, implementation, and validation results. The deliverable report can be found in [D5.5.4 Robot Navigation](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.4.pdf).

# Run the Robot Navigation Test


### Steps
1. **Install the required software components:**
   
   Set up the development environment for testing the Pepper robot navigation system. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

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

3. **Update Test Configuration File:**
   
   Navigate to the test configuration file located at `$HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotNavigationTest/config/robotNavigationTestConfiguration.ini` and update the configuration according to your testing needs:
   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;"> You can choose the tests to run separately or you can enable all of them at once. </span>
      </div>

   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `pathPlanningBfs` | Enable BFS algorithm tests | `true`, `false` | `true` |
   | `pathPlanningDijkstra` | Enable Dijkstra algorithm tests | `true`, `false` | `true` |
   | `pathPlanningAstar` | Enable A* algorithm tests | `true`, `false` | `true` |
   | `serviceTests` | Enable navigation service tests | `true`, `false` | `true` |
   | `boundaryTests` | Enable boundary condition tests | `true`, `false` | `true` |
   | `configurationTests` | Enable configuration tests | `true`, `false` | `true` |
   | `algorithmComparison` | Enable performance comparison (Time) | `true`, `false` | `true` |
   | `verboseMode` | Diagnostic info printing | `true`, `false` | `true` |

4. **Run the `robotNavigationTest`:**

   **Approach 1: Launch file based Test Execution (Recommended)**
   
   Run the complete test suite using two separate terminals:
    
   - **Terminal 1** - Launch robot interface: Make sure you change the robot IP, and network interface in the launch file to your specific robots's IP address and network interface name.
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        roslaunch unit_tests robotNavigationTestLaunchRobot.launch
        ```

        You can select the robot to be used by using the `robot_ip` parameter when running the above command. Other optional parameters that can be used are: `robot_port`, `roscore_ip`, and `network_interface`. An example is shown below.

      ```bash
      roslaunch unit_tests robotNavigationTestLaunchRobot.launch robot_ip:=172.29.111.240
      ```

   - **Terminal 2** - Launch test harness:(**You can set the initial (home) coordinate of robot x, y, and theta values in the launch file argument parameters, if absolute localization is not fully implemented** )
        ```bash
        cd $HOME/workspace/pepper_rob_ws 
        source devel/setup.bash
        roslaunch unit_tests robotNavigationTestLaunchTestHarness.launch
        ```

      <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">The test harness automatically starts all required components including <code>robotNavigationDriver (which works as a robot localization providing a /robotLocalization/pose topic)</code>, <code>robotNavigation</code>, and <code>robotNavigationTest</code> nodes. The system waits for all services to be available before beginning test execution. </span>
      </div>

   **Approach 2: Manual Step-by-Step Execution**
   
   For individual component testing and debugging, run each component in separate terminals:
   
   - **Terminal 1** - Launch robot interface: 
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        roslaunch unit_tests robotNavigationLaunchTestRobot.launch
        ```
        
   - **Terminal 2** - Start robot localization:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun cssr_system robotLocalization
        ```
        
   - **Terminal 3** - Start robot navigation:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun cssr_system robotNavigation
        ```
        
   - **Terminal 4** - Set initial robot pose: Assign the current known robot coordinate (the home location)
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosservice call /robotLocalization/set_pose 2.0 7.8 270.0
        ```
        
   - **Terminal 5** - Run navigation tests:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        source devel/setup.bash
        rosrun unit_tests robotNavigationTest
        ```

      <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
      </div>

5. **Using the Test Suite and Monitoring Results:**
   Upon launching the test suite, the following components are automatically initialized and tested:

   **Verify Test Services**
   Confirm all required services are available:
   ```bash
   rosservice list | grep -E "(set_goal|set_pose)"
   ```
   
   **Monitor Test Execution**
   View real-time test progress and pose updates:
   ```bash
   rostopic echo /robotLocalization/pose
   ```
   
   **Manual Pose Setting** (for debugging):
   Reset the robot's pose during testing:
   ```bash
   rosservice call /robotLocalization/set_pose 2.0 7.8 270.0
   ```
   
   **Manual Navigation Testing**
   Test navigation service directly:
   - Goal Location (<code>goal_x</code>, <code>goal_y</code>)
       - <code>goal_x</code>: X-coordinate of the goal location in meters
       - <code>goal_y</code>: Y-coordinate of the goal location in meters
   - Goal Orientation (<code>goal_theta</code>)    
       - <code>goal_theta</code>: Final orientation of the robot at the goal location in degrees
   
   - Sample Test Navigation (Navigate to position (2.0, 6.0) with 270-degree orientation):
   ```bash
   rosservice call /robotNavigation/set_goal 2.0 6.0 270.0
   ```
   
   - If **negative** value of theta needs to be passed as a goal, you have to use the following format:
   ```bash
   rosservice call /robotNavigation/set_goal -- 2.0 7.8 -45.0
   ```
   ```bash
   rosservice call /robotNavigation/set_goal "{goal_x: 2.0, goal_y: 7.8, goal_theta: -45.0}"
   ```

## Test Results and Output

The test suite generates comprehensive reports and visualizations:

**Test Report Location:**
```bash
$HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotNavigationTest/data/robotNavigationTestOutput.dat
```


**Test Result Output:**
```
Robot Navigation Test Report: 
==========================================
Date: 2025-06-03 11:46:32
Environment Map: scenarioOneEnvironmentMap.dat
Configuration Map: scenarioOneConfigMap.dat
Robot Topics: pepperTopics.dat
Verbose Mode: true
Test Method: Service-based testing

BFS Path Planning Test 1: PASS
	Algorithm: BFS
	Start: (2, 7.8, 270)
	Goal: (2, 6, 270)

BFS Path Planning Test 2: PASS
	Algorithm: BFS
	Start: (2, 6, 270)
	Goal: (2, 6, 0)

Dijkstra Path Planning Test 1: PASS
	Algorithm: Dijkstra
	Start: (2, 7.8, 270)
	Goal: (2.6, 6, 270)

Dijkstra Path Planning Test 2: PASS
	Algorithm: Dijkstra
	Start: (2.6, 6, 270)
	Goal: (2, 6, 45)

A* Path Planning Test 1: PASS
	Algorithm: A*
	Start: (2, 7.8, 270)
	Goal: (2.6, 6, 270)

A* Path Planning Test 2: PASS
	Algorithm: A*
	Start: (2.6, 6, 270)
	Goal: (2, 6, 45)

Boundary Test 1 (Goal: 10.000000, 10.000000): PASS
	Goal: (10, 10, 0)

Boundary Test 2 (Goal: -1.000000, 2.000000): PASS
	Goal: (-1, 2, 0)

Boundary Test 3 (Goal: 2.000000, -1.000000): PASS
	Goal: (2, -1, 0)

Boundary Test 4 (Goal: 0.000000, 0.000000): PASS
	Goal: (0, 0, 0)

Boundary Test 5 (Goal: 7.000000, 7.000000): PASS
	Goal: (7, 7, 0)

Service Test 1 (Goal: 2.000000, 7.000000, 270.000000): PASS
	Service Call: /robotNavigation/set_goal
	Goal: (2, 7, 270)

Service Test 2 (Goal: 2.000000, 6.000000, 0.000000): PASS
	Service Call: /robotNavigation/set_goal
	Goal: (2, 6, 0)

bfs Performance Test:
	Execution Time: 14747 ms
	Path Found: Yes
	Estimated Waypoints: 10

dijkstra Performance Test:
	Execution Time: 13660 ms
	Path Found: Yes
	Estimated Waypoints: 10

astar Performance Test:
	Execution Time: 13944 ms
	Path Found: Yes
	Estimated Waypoints: 10

==========================================
Test Execution Summary
==========================================
Overall Result: ALL TESTS PASSED
Test report location: /home/br/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotNavigationTest/data/robotNavigationTestOutput.dat
```

**Performance Metrics:** The path choosen is a short path, the performance will vary based on the length.
| Algorithm | Execution Time | Path Found | Status |
|-----------|---------------|------------|---------|
| **BFS**      | ~14.7s |  Yes |  PASSED |
| **Dijkstra** | ~13.7s |  Yes |  PASSED |
| **A-Star**   | ~13.9s |  Yes |  PASSED |

**Generated Visualization Files:**
- `environmentMapWaypointsAstar.png` - A* algorithm waypoints
- `configMapWaypointsBFS.png` - BFS algorithm waypoints  
- `environmentMapWaypointsDijkstra.png` - Dijkstra algorithm waypoints

## Troubleshooting

**Common Issues and Solutions:**

1. **Test Failures due to Pose Issues:**
   ```bash
   # Verify pose subscription
   rosnode info /robotNavigation | grep Subscriptions
   # Should show: /robotLocalization/pose [geometry_msgs/Pose2D]
   ```

2. **Service Unavailability:**
   ```bash
   # Check service status
   rosservice list | grep robotNavigation
   # Restart navigation system if needed
   ```

3. **Test Timeout Issues:**
   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
   <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
   <span style="color: #cccccc;">If tests run slowly or timeout, consider reducing test scenarios in the configuration file or increasing timeout values. Individual algorithm tests typically take 13-15 seconds each. </span>
   </div>

4. **Verification Commands:**
   ```bash
   # Check all required nodes
   rosnode list | grep -E "(robotNavigation|robotLocalization)"
   
   # Monitor test logs
   rostopic echo /rosout | grep robotNavigationUnitTest
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