
# Animate Behaviour Unit Test for ROS

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the animateBehaviour ROS node within the CSSR4Africa project. The node enhances robot interactions by creating a lifelike presence through subtle, continuous movements. It achieves this by generating random patterns of body movements, hand flexing, and base rotation while keeping joint positions close to their default home values for robotic or simulator platforms. The tests verify the correct execution of animations in various modes: animate behaviours, hands, body, rotation, or all components combined.

# Documentation
Accompnaying this code, there are deliverable reports that provides a detailed explanation of the code and how to run the tests. The deliverable reports are can be found in [D5.2 Animate Behaviour Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.2.pdf)

# Run the Animate Behaviour Unit Test 
## Physical Robot 
### Steps
1.**Install the required software components:**
 Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

2. **Clone and build the project (if not already cloned)**
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
       
3. **Update Configuration File:**
   - Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`and update the configuration according to the key-value pairs below:

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
  
    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.2.pdf" style="color: #66b3ff;">D5.2 Animate Behaviour Manual</a>. Otherwise, the preferred values are the ones already set in the `writeInitialConfigurationFile` function.</span>
  </div>

4.  **Run the `animateBehaviourTest` from the`unit_tests`  package**.
      Follow below steps, run in different terminals.
      -  Source the workspace in first terminal:
          ```bash
            cd $HOME/workspace/pepper_rob_ws
            source devel/setup.bash
          ```
      -  Launch the robot:
          ```bash
            cd .. && roslaunch unit_tests animateBehaviourLaunchRobot.launch robot_ip:=172.29.111.230 network_interface:=wlp0s20f3
          ```
          <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP address <code>172.29.111.240</code> and the network interface <code>wlp0s20f3</code> are correctly set based on your robot's configuration and your computer's network interface. </span></div>

      - Open a new terminal to launch the animate behavior node and the unit test node.
          ```bash
            cd $HOME/workspace/pepper_rob_ws
            cd .. && source devel/setup.bash
            cd .. && roslaunch unit_tests animateBehaviourLaunchTestHarness.launch
        ```
5.  **Test result**
    The test reports are recorded under `~/workspace/pepper_rob_ws/src/unit_tests/animateBehaviourTest/data/animateBehaviourTestOutput.dat`

## Simulator Robot
### Steps
1.**Install the required software components:**
  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

2. **Clone and build the project (if not already cloned)**:
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
       
3. **Update Configuration File:**
   - Navigate to `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`:
   
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
  
    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.2.pdf" style="color: #66b3ff;">D5.2 Animate Behaviour Manual</a>. Otherwise, the preferred values are the ones already set in the `writeInitialConfigurationFile` function.</span>

4. **Run the `animateBehaviourTest` from the `unit_tests`  package**. 
     Follow below steps, run in different terminals. 
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws
          source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          cd .. && roslaunch unit_tests animateBehaviourLaunchRobot.launch robot_ip:=172.29.111.240 network_interface:=wlp0s20f3
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP address <code>172.29.111.240</code> and the network interface <code>wlp0s20f3</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>

    - Open a new terminal to launch the animate behavior node and the unit test node.
        ```bash
          cd $HOME/workspace/pepper_sim_ws
          cd .. && source devel/setup.bash
          cd .. && roslaunch unit_tests animateBehaviourLaunchTestHarness.launch
        ```
  
  5.  **Test result**
    The test reports are recorded under `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/data/animateBehaviourTestOutput.dat`

## Tests Executed
### Test 1
  Test 1 evaluates the hand behavior of the animate behavior and executes flexible hand movement. To test it, set the behavior value to `hands` in the configuration file located at `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`. The test results are as follows:  
      ===============================================================================================
      === New Test Run Started at 2025-01-10 11:37:47 ===
      ===============================================================================================

      ===============================================================================================
      Animate behaviour enabled: PASSED
      ...............................................................................................
      Test 1: Test Flexi Hand Animate Behavior

              Configuration Settings:
              armMaximumRange     : 0.2,0.2,0.2,0.35,0.2
              behaviour           : hands
              gestureDuration     : 1.0
              handMaximumRange    : 0.7
              legMaximumRange     : 0.1,0.1,0.08
              legRepeatFactor     : 8
              numPoints           : 100
              numPointsLeg        : 2
              platform            : robot
              robotTopics         : pepperTopics.dat
              rotMaximumRange     : 0.3
              selectedRange       : 0.5
              simulatorTopics     : simulatorTopics.dat
              verboseMode         : false

              Flexi movement started: PASSED

              Joint names:
                  left arm: ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"]
                  left hand: ["LHand"]
                  right arm: ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
                  right hand: ["RHand"]

              Ensure the joint moves to the Home position before starting random movements.
              The values of the home positions are:
                  left arm: [1.5625, 0.0997, -0.3434, -1.715, 0.06592] 
                  left hand: [0.6695] 
                  right arm: [1.541, -0.09664, 0.30664, 1.6981, -0.05679] 
                  right hand: [0.66608] 

              After the joint is in the home position, start moving to random positions continuously.
              The random positions captured are:
                  left arm: 
                  [1.5625,0.0997,-0.292275,-1.715,-0.0738872] 
                  [1.5625,0.0997,-0.372787,-1.715,-0.00163233] 
                  [1.5625,0.0997,-0.319568,-1.715,-0.0602867] 
                  [1.5625,0.0997,-0.344443,-1.715,-0.0410555] 
                  [1.5625,0.0997,-0.328404,-1.715,-0.0965686] 
                  [1.5625,0.0997,-0.29347,-1.715,0.17709] 
                  [1.5625,0.0997,-0.321619,-1.715,0.00870356] 
                  [1.5625,0.0997,-0.382202,-1.715,0.0790169] 
                  [1.5625,0.0997,-0.362397,-1.715,-0.092998] 
                  [1.5625,0.0997,-0.333815,-1.715,0.236559] 
                  [1.5625,0.0997,-0.332182,-1.715,0.0810651] 
                  [1.5625,0.0997,-0.396142,-1.715,-0.0729039] 
                  [1.5625,0.0997,-0.414343,-1.715,0.0137635] 
                  [1.5625,0.0997,-0.27347,-1.715,0.0189709] 
                  [1.5625,0.0997,-0.318506,-1.715,0.151056] 
                  [1.5625,0.0997,-0.269514,-1.715,-0.0210742] 
                  [1.5625,0.0997,-0.398779,-1.715,0.23927] 
                  [1.5625,0.0997,-0.287537,-1.715,0.163829] 
                  [1.5625,0.0997,-0.331879,-1.715,0.00579368] 
                  [1.5625,0.0997,-0.293659,-1.715,0.0679609] 

                  left hand: 
                  [0.651182] 
                  [0.746912] 
                  [0.820731] 
                  [0.831668] 
                  [0.552954] 
                  [0.823442] 
                  [0.714971] 
                  [0.795861] 
                  [0.742381] 
                  [0.665403] 
                  [0.59713] 
                  [0.830479] 
                  [0.6702] 
                  [0.836283] 
                  [0.500339] 
                  [0.810445] 
                  [0.569015] 
                  [0.807128] 
                  [0.60649] 
                  [0.70249] 

                  right arm: 
                  [1.541,-0.09664,0.264806,1.6981,0.103489] 
                  [1.541,-0.09664,0.361577,1.6981,-0.144075] 
                  [1.541,-0.09664,0.229336,1.6981,-0.000794754] 
                  [1.541,-0.09664,0.283598,1.6981,-0.193417] 
                  [1.541,-0.09664,0.327467,1.6981,0.0847753] 
                  [1.541,-0.09664,0.297641,1.6981,-0.091553] 
                  [1.541,-0.09664,0.276529,1.6981,0.0292464] 
                  [1.541,-0.09664,0.278074,1.6981,-0.124647] 
                  [1.541,-0.09664,0.264051,1.6981,-0.157105] 
                  [1.541,-0.09664,0.316451,1.6981,-0.0110009] 
                  [1.541,-0.09664,0.30004,1.6981,-0.110694] 
                  [1.541,-0.09664,0.229374,1.6981,-0.0892808] 
                  [1.541,-0.09664,0.317356,1.6981,0.0456078] 
                  [1.541,-0.09664,0.327973,1.6981,0.0325273] 
                  [1.541,-0.09664,0.32753,1.6981,-0.192552] 
                  [1.541,-0.09664,0.262877,1.6981,-0.16422] 
                  [1.541,-0.09664,0.231383,1.6981,-0.155829] 
                  [1.541,-0.09664,0.371565,1.6981,0.112077] 
                  [1.541,-0.09664,0.353419,1.6981,0.0887051] 
                  [1.541,-0.09664,0.311272,1.6981,-0.0732257] 

                  right hand: 
                  [0.806434] 
                  [0.826068] 
                  [0.631274] 
                  [0.568022] 
                  [0.546472] 
                  [0.712125] 
                  [0.547966] 
                  [0.590914] 
                  [0.652124] 
                  [0.640521] 
                  [0.631965] 
                  [0.573123] 
                  [0.521069] 
                  [0.621963] 
                  [0.756075] 
                  [0.748536] 
                  [0.75746] 
                  [0.666206] 
                  [0.613544] 
                  [0.514266] 

              Flexi movement ended: PASSED
      ...............................................................................................

      Animate behaviour disabled: PASSED
      ===============================================================================================

      ===============================================================================================
      Test Run Completed with Result: PASSED
      ===============================================================================================



### Test 2
  Test 2 evaluates the body behavior of the animate behavior and executes subtle body movement. To test it, set the behavior value to `body` in the configuration file located at `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`. The test results are as follows:  
      ===============================================================================================
      === New Test Run Started at 2025-01-10 11:40:21 ===
      ===============================================================================================



      ===============================================================================================
      Animate behaviour enabled: PASSED
      ...............................................................................................
      Test 2: Test Subtle Body Animate Behavior
              Configuration Settings:
              armMaximumRange     : 0.2,0.2,0.2,0.35,0.2
              behaviour           : body
              gestureDuration     : 1.0
              handMaximumRange    : 0.7
              legMaximumRange     : 0.1,0.1,0.08
              legRepeatFactor     : 8
              numPoints           : 100
              numPointsLeg        : 2
              platform            : robot
              robotTopics         : pepperTopics.dat
              rotMaximumRange     : 0.3
              selectedRange       : 0.5
              simulatorTopics     : simulatorTopics.dat
              verboseMode         : false

              Subtle body movement started: PASSED

              The joints used for the subtle body movements are:
                  left arm: ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"]
                  leg: ["HipPitch", "HipRoll", "KneePitch"]
                  right arm: ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
                  right hand: ["RHand"]

              Ensure the joint moves to the Home position before starting random movements.
              The values of the home positions are:
                  left arm: [1.7625, 0.0997, -0.1334, -1.715, 0.06592] 
                  leg: [-0.0107, -0.00766, 0.03221] 
                  right arm: [1.741, -0.09664, 0.09664, 1.6981, -0.05679] 
                  right hand: [0.6695] [0.66608] 

              After the joint is in the home position, start moving to random positions continuously.
              The random positions captured are:
                  left arm: 
                  [1.62382,0.0596369,-0.279035,-1.35193,0.130326] 
                  [1.57964,0.0799804,-0.362658,-1.57433,0.212618] 
                  [1.61693,0.023157,-0.313,-1.62983,0.23621] 
                  [1.7273,0.0861022,-0.31242,-2.02871,0.0557337] 
                  [1.61689,0.0524975,-0.40194,-1.48101,0.0211995] 
                  [1.4058,0.0710829,-0.282493,-1.52123,0.208936] 
                  [1.55773,0.0818875,-0.410497,-1.8107,-0.0478896] 
                  [1.3677,0.123944,-0.333529,-2.04362,0.226352] 
                  [1.54562,0.0314147,-0.351383,-1.89839,0.0575908] 
                  [1.58075,0.120922,-0.420453,-1.70584,0.176971] 
                  [1.68103,0.121464,-0.358992,-1.408,0.0852411] 
                  [1.58016,0.0435781,-0.397034,-1.58176,0.162348] 
                  [1.4579,0.0972499,-0.404204,-2.00023,0.112549] 
                  [1.46417,0.102902,-0.400524,-1.86592,0.0651056] 
                  [1.52602,0.0520344,-0.419368,-1.60438,0.245496] 
                  [1.73204,0.123166,-0.35658,-1.65177,0.143133] 
                  [1.49503,0.146776,-0.275667,-1.82733,-0.0269436] 
                  [1.36816,0.0669359,-0.280615,-1.78845,0.124969] 
                  [1.66585,0.051208,-0.268122,-2.07429,0.102693] 
                  [1.67174,0.0692576,-0.4135,-1.8521,-0.0899963] 

                  leg: 
                  [-0.131925,0.018079,0.0285611] 
                  [-0.0985757,-0.0113764,0.0138817] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.148327,-0.0173623,0.0522716] 
                  [-0.11641,-0.0279676,0.0394486] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 
                  [-0.1107,-0.00766,0.03221] 

                  right arm: 
                  [1.46769,-0.01919,0.295201,1.53844,-0.0129773] 
                  [1.43642,-0.021992,0.305678,1.60704,0.0608267] 
                  [1.67205,-0.0823008,0.229754,1.38765,-0.111334] 
                  [1.47882,-0.0710025,0.305417,1.47666,-0.151325] 
                  [1.3357,-0.0539978,0.352086,1.46762,-0.0687907] 
                  [1.60439,-0.165364,0.336697,1.36586,-0.0680843] 
                  [1.59794,-0.075546,0.3765,1.82122,0.0358486] 
                  [1.50883,-0.148712,0.27954,1.38746,-0.0931274] 
                  [1.72894,-0.131763,0.371321,1.70232,-0.0952553] 
                  [1.62376,-0.0915906,0.372383,2.03503,-0.192382] 
                  [1.43258,-0.173113,0.248205,2.02962,0.089785] 
                  [1.60218,-0.0745816,0.277143,1.90639,-0.199014] 
                  [1.71692,-0.150668,0.371238,1.42779,-0.0314418] 
                  [1.64534,-0.0909448,0.297652,1.75097,-0.0898805] 
                  [1.44568,-0.0410617,0.295132,1.77248,-0.20346] 
                  [1.50307,-0.10324,0.333449,1.36779,-0.0387863] 
                  [1.41034,-0.0208153,0.319571,1.46266,0.0808531] 
                  [1.46675,-0.130802,0.339353,1.6103,-0.112383] 
                  [1.70529,-0.144346,0.297415,1.94545,-0.129619] 
                  [1.45216,-0.16097,0.359954,1.87391,0.0974557] 

                  right hand: 
                  [0.818253] 
                  [0.72883] 
                  [0.818376] 
                  [0.631128] 
                  [0.659163] 
                  [0.619567] 
                  [0.729619] 
                  [0.790729] 
                  [0.651486] 
                  [0.558672] 
                  [0.553099] 
                  [0.658264] 
                  [0.829085] 
                  [0.620266] 
                  [0.521598] 
                  [0.622409] 
                  [0.790659] 
                  [0.790659] 
                  [0.549032] 
                  [0.742073] 
                  [0.607318] 

              Subtle body movement ended: PASSED
      ...............................................................................................
      Animate behaviour disabled: PASSED
      ===============================================================================================

      ===============================================================================================
      Test Run Completed with Result: PASSED
      ===============================================================================================

### Test 3
  Test 3 evaluates the rotation behavior of the animate behavior and executes rotation about the robot’s z-axis. To test it, set the behavior value to `rotation` in the configuration file located at `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`. The test results are as follows:  
      ===============================================================================================
      === New Test Run Started at 2025-01-10 11:42:44 ===
      ===============================================================================================



      ===============================================================================================
      Animate behaviour enabled: PASSED
      ...............................................................................................
      Test 3: Test Rotation in Z-axis Animate Behavior
              Configuration Settings:
              armMaximumRange    : 0.2,0.2,0.2,0.35,0.2
              behaviour          : rotation
              gestureDuration    : 1.0
              handMaximumRange   : 0.7
              legMaximumRange    : 0.1,0.1,0.08
              legRepeatFactor    : 8
              numPoints          : 100
              numPointsLeg       : 2
              platform           : robot
              robotTopics        : pepperTopics.dat
              rotMaximumRange    : 0.3
              selectedRange      : 0.5
              simulatorTopics    : simulatorTopics.dat
              verboseMode        : false

              Rotation in the z-axis started: PASSED

              The joints used for the rotation in the z-axis are:
                  base: ["Wheels"]

              Ensure the joint moves to the Home position before starting random movements.
              The values of the home positions are:
                  base: 2.000000 

              After the joint is in the home position, start moving to random positions continuously.
              The random position values generated are:
                  0.005953
                  -0.256519
                  -0.091839
                  0.105437
                  -0.020657
                  0.070199
                  -0.026131
                  -0.145514
                  0.12984
                  0.175822
                  0.224146
                  -0.028179
                  0.141522
                  -0.070674

              Rotation in the z-axis ended: PASSED
      ...............................................................................................
      Animate behaviour disabled: PASSED

      ===============================================================================================
      Test Run Completed with Result: PASSED
      ===============================================================================================
  
### Test 4
  Test 4 executes the combination of all behavior types at once (hands, body, rotation). To test it, set the behavior value to `All` in the configuration file located at `~/workspace/pepper_sim_ws/src/unit_tests/animateBehaviourTest/config/animateBehaviourTestConfiguration.ini`. The test results are as follows: 
      ===============================================================================================
      === New Test Run Started at 2025-01-10 12:02:00 ===
      ===============================================================================================

      ===============================================================================================
      Test 4: Test All Animate Behavior
      -----------------------------------------------------------------------------------------------
      Initialization:
        Animate behaviour enabled: PASSED

      Individual Movement Status:
        Flexi Hand Movement:
          Started: PASSED
          Ended:   PASSED

        Subtle Body Movement:
          Started: PASSED
          Ended:   PASSED

        Rotation Movement:
          Started: PASSED
          Ended:   PASSED

      Overall Status:
        All movements started: PASSED
        All movements ended:   PASSED
        Behaviour disabled:    PASSED
      -----------------------------------------------------------------------------------------------
      Final Result: PASSED
      ===============================================================================================


      ===============================================================================================
      Test Run Completed with Result: PASSED
      ===============================================================================================

 
  

## Supporthand 

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ebirhan@andrew.cmu.edu">ebirhan@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>




## License  


Date:   2025-01-10