<div align="center">
  <h1>Robot Mission Interpreter Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `behaviorController` node within the CSSR4Africa project (`cssr_system` package). The node evaluates the execution of the currently set robot mission specification, validating each of the action and condition nodes utilized.

 The output of the test is displayed on the screen and also stored at `~/workspace/pepper_rob_ws/src/unit_tests/behaviorControllerTest/data/behaviorControllerTestOutput.dat`.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.4.3 Robot Mission Interpreter](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf).

# Run the Robot Mission Interpreter Test 
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
   
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/behaviorControllerTest/config/behaviorControllerTestConfiguration.ini` and update the configuration according to the key-value pairs below:

| Key                   |     Value |
| --------------------- |     ------------------- |
| `verboseMode`             |      <true/false - enables/disables the display of diagnostic messages>   |
| `failureRate`           |      the rate at which service calls and topics will provide a failed(not successful) response |
| `arrivalRate`          |      the rate at wich an event occurs( valid only for the driver functions) |
| `traversal`          |      which path of the behavior tree to test |
  

<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf" style="color: #66b3ff;">D5.4.3 Robot Mission Interpreter</a>. Otherwise, the preferred values are the ones already set in the `behaviorControllerTestConfiguration.ini` file.</span>
  </div>

 You can also update other keys of that configuration file. For more details, refer to <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.4.3.pdf" style="color: #66b3ff;">D5.4.3 Robot Mission Interpreter</a>.

4. **Run the `behaviorControllerTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests behaviorControllerTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the behaviorControllerTest (which launches the behaviorController node and run tests on it). This creates a driver for the
        - `/animateBehaviour/setActivation`
        - `/gestureExecution/perform_gesture`
        - `/overtAttention/set_mode`
        - `/robotLocalization/reset_pose`
        - `/robotNavigation/set_goal`
        - `/speechEvent/set_language`
        - `/speechEvent/set_enabled`
        - `/tabletEvent/prompt_and_get_response`
        - `/textToSpeech/say_text`
    <br/>services, and the 
        - `/overtAttention/mode`
        - `/speechEvent/text`
    <br/> topics.
    
     or launches the actual nodes (if available).
    ```bash
    cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch unit_tests behaviorControllerTestLaunchHarness.launch launch_drivers:=true launch_test:=true
    ```
    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
    <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
    <span style="color: #cccccc;">Setting the argument <code>launch_test</code> to <code>true</code> runs the tests based on the configuration. Setting the argument <code>launch_drivers</code> to <code>true</code> launches the drivers and stubs required to drive the <code>behaviorController</code> node from the <code>unit_tests</code> package, while setting it to <code>false</code> launches the actual nodes (if available).
    </div>

## Tests Executed
The tests retrieve the status of each action and condition nodes available in the `behaviorController` ROS node, that also form part of the robot mission specification file that has been configured to run.

### Traversal

In the `behaviorControllerTest`, the `traversal` configuration key determines which part of the mission specification (a behavior tree) should be executed and tested. Since the mission is structured as a tree, this effectively selects which sub-path of the tree to traverse during testing.

For the **Lab Tour** scenario, there are four predefined **partial paths**, each representing a specific failure condition, as well as one **complete** path representing a successful execution.

| Key         | Description                                                                 | Failing Condition Node     |
|-------------|-----------------------------------------------------------------------------|----------------------------|
| `partial01` | Visitor is detected but mutual gaze is not established                     | `IsMutualGazeDiscovered`   |
| `partial02` | Visitor does not respond when asked if they would like a tour              | `HasVisitorResponded`      |
| `partial03` | Visitor explicitly declines the offer to have a tour                       | `IsVisitorResponseYes`     |
| `partial04` | Visitor leaves midway through the tour                                     | `IsVisitorPresent`         |

The `complete` path corresponds to a successful tour execution, where no failures occur.

By default, the `traversal` configuration is set to:  
```yaml
traversal: complete
```
If any of the action or condition nodes that are part of the execution flow of the selected mission specification file fail, the `behaviorControllerTest` application will identify and log it. A sample of the result of such a test is presented below.


## Results
The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/behaviorControllerTest/data/behaviorControllerTestOutput.dat` file. It contains the name of the mission node that was executed by the `behaviorController` and whether it suceeded or not. Below is the output of the from the test of the `lab_tour` robot mission specifiction.
```
Test Results
Date: 2025-06-01 18:20:25

StartOfTree -> Passed!
SetSpeechEvent -> Passed!
RetrieveListOfExhibits -> Passed!
SetRobotPose -> Passed!
SetAnimateBehavior -> Passed!
SetOvertAttentionMode -> Passed!
IsVisitorDiscovered -> Passed!
SayText -> Passed!
PerformIconicGesture -> Passed!
IsMutualGazeDiscovered -> Passed!
GetVisitorResponse -> Passed!
HasVisitorResponded -> Passed!
IsVisitorResponseYes -> Passed!
IsListWithExhibit -> Passed!
SelectExhibit -> Passed!
Navigate -> Passed!
IsVisitorPresent -> Passed!
DescribeExhibit -> Passed!
PerformDeicticGesture -> Passed!

```
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ttefferi@andrew.cmu.edu">ttefferi@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-04-26