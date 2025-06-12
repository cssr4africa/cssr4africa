<div align="center">
  <h1>Environment Map Generation Unit Test</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `mapGenerationTest` provides comprehensive unit tests for the `mapGeneration` node within the CSSR4Africa project (`unit_tests` package). This test suite verifies that the map generation system correctly processes obstacle data, generates workspace maps, and creates configuration space maps that account for the robot's physical dimensions. The test results are logged in the file `$HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/mapGenerationTest/data/test_output/test_output.logs`.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D5.5.3 Environment Map Generation](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.3.pdf).

# Test Validation

## Communication and Computation Functionality Validation

The unit tests validate the communication and computation functionality by testing the complete input-to-output data flow of the mapGeneration node. The validation process follows this methodology:

### Input Data Sources:
- **mapGenerationInput.dat**: Defines map dimensions (6.80 x 9.93 m), obstacle file references, and output filenames
- **testObstacles.dat**: Contains obstacle definitions with precise coordinates and dimensions
- **Configuration files**: Define robot parameters including radius, resolution, and operational mode

### Expected Output Data (Sink):
- **environmentMap.png**: Grayscale image (6.80 x 9.93 m) showing workspace with obstacles
- **configurationSpaceMap.png**: Grayscale image showing robot configuration space with expanded obstacles

### Validation Criteria:
The tests verify that the mapGeneration node produces correct output data by validating:

1. **File Generation**: Both output PNG files are created successfully
2. **Dimension Accuracy**: Output images match expected dimensions (680x993 pixels)
3. **Pixel Content Validation**: 
   - Environment map contains obstacles (black pixels, value=0) and free space (white pixels, value=255)
   - Configuration space contains more obstacle pixels than environment map due to robot radius expansion
4. **Obstacle Expansion Verification**: Configuration space obstacles are geometrically larger than environment obstacles by the robot radius plus clearance

### Input-Output Mapping:
```
Input: OBSTACLE 2.5775 3.12 2.315 1.44 (rectangle at 2.58m, 3.12m with dimensions 2.32m x 1.44m)
↓ Processing: Convert to pixels (100 pixels/meter), place in 680x993 grid
↓ Robot radius expansion: Dilate by robot radius + clearance
Output: Black pixels in environment map + expanded black pixels in configuration space
```

## Configuration Functionality Validation

The tests validate that changes in configuration parameters produce the expected behavioral changes:

### Robot Radius Parameter Testing:
**Configuration Change**: `robotRadius` values of 0.1m, 0.3m, and 0.5m
**Expected Behavior**: Larger robot radius results in more conservative configuration space
**Validation Method**: 
- Test measures obstacle pixel count in configuration space for each radius
- Verifies: `obstacles(0.1m) < obstacles(0.3m) < obstacles(0.5m)`
- Example results: 364,587 → 444,107 → 518,938 obstacle pixels

### Verbose Mode Parameter Testing:
**Configuration Change**: `verboseMode` set to `true` vs `false`
**Expected Behavior**: 
- `verboseMode=true`: Node displays maps
- `verboseMode=false`: Node runs without user interaction
**Validation Method**: Test monitors execution time and verifies successful completion in both modes

### Resolution Parameter Impact:
**Configuration Change**: `resolution` value affects pixel-to-meter conversion
**Expected Behavior**: Different resolutions change map detail level
**Validation Method**: Tests verify correct dimension calculations based on resolution

### Mode Parameter Testing:
**Configuration Change**: `mode` set to "CAD" vs "SLAM"
**Expected Behavior**: 
- CAD mode: Uses predefined obstacle data
- SLAM mode: Would use sensor data (currently not implemented)
**Validation Method**: Tests verify CAD mode functionality and detect unsupported SLAM mode (mapGeneration node fails with exit code: 256 because SLAM mode is not Implemented yet)

# Run the Map Generation Test

## Setup and Execution 
### Steps
1. **Install the required software components:**
   
   Set up the development environment for the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

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
      ```
      ```bash
      catkin_make
      ```
      ```bash
      source devel/setup.bash 
      ```
    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
        <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
        <span style="color: #cccccc;"> Make sure you have the <code>cssr4africa/unit_tests/mapGenerationTest</code> package built successfully before proceeding. </span>
    </div>

3. **Configure test data**:
   - The test uses data files located in `unit_tests/mapGenerationTest/data/`:
     - `mapGenerationInput.dat`: Defines map dimension and filenames
     - `testObstacles.dat`: Contains obstacle definitions
     - `testRobotRadius.dat`: Contains robot radius values for configuration space tests

   - Sample `mapGenerationInput.dat` contents:
     ```
      mapWidth                  6.80
      mapHeight                 9.93
      obstacleFile              obstacles.dat
      environmentMapFile        environmentMap.png
      configurationSpaceMapFile configurationSpaceMap.png
     ```

   - Sample `testObstacles.dat` contents:
     ```
      OBSTACLE   2.5775 3.12  2.315 1.44
      OBSTACLE   1.3  0.45  2.60 0.90
      OBSTACLE   2.20  0.30  4.40 0.60
      OBSTACLE   3.31  1.925  0.22 0.25
      OBSTACLE   3.31  5.525  0.22 0.25
      OBSTACLE   3.31  7.325  0.22 0.25
      OBSTACLE   3.31  9.125  0.22 0.25
      OBSTACLE   0.40  4.965  0.80 9.93
      OBSTACLE   3.40  9.465  6.80 0.93
      OBSTACLE   6.20  4.965  1.20 9.93
     ```

4. **Run the `mapGenerationTest` from the `unit_tests` package:**
   It's important to note that the mapGeneration node has been designed to operate independently of the physical robot hardware. Users can generate environment maps and configuration space maps without needing to connect to a Pepper robot. Since the CAD-based map generation relies solely on geometric data provided in the input files, no sensor data or robot connectivity is required. This makes the node useful for pre-planning environments before deploying the robot, and enables development and testing on any computer with **`ROS installed and roscore running`**.

   You can run the test in two ways:

   - Using rosrun:
     ```bash
     cd $HOME/workspace/pepper_rob_ws
     source devel/setup.bash
     rosrun unit_tests mapGenerationTest
     ```

   - Using roslaunch (recommended):
     ```bash
     cd $HOME/workspace/pepper_rob_ws
     source devel/setup.bash
     roslaunch unit_tests mapGenerationTestLaunchTestHarness.launch
     ```
     <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
        <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">The test doesn't require a physical robot to run, as it focuses on the map generation functionality rather than robot movement. </span>
     </div>

5. **Tests Executed**
   
   The test suite executes the following tests:

   - **Basic Map Generation Test**:  
     Tests the core functionality by launching the actual mapGeneration node with standard configuration. Validates that both environment and configuration space maps are generated with correct dimensions and that the configuration space contains more obstacles than the environment map due to robot radius expansion.

   - **Different Robot Radii Test**:  
     Systematically tests robot radii of 0.1m, 0.3m, and 0.5m to verify that larger robot radii produce more conservative configuration spaces. This test validates the geometric correctness of the obstacle expansion algorithm.

   - **Verbose Mode Configuration Test**:  
     Tests the verboseMode configuration parameter to ensure the node behaves correctly in both interactive (`verboseMode=true`) and automated (`verboseMode=false`) modes.

6. **Test Results**

   The results of the tests are logged in the `unit_tests/mapGenerationTest/data/test_output/test_output.logs` file and printed to the console. 

   ### Expected Test Output Analysis:
   
   **Successful Test Execution Indicators:**
   - All tests show `[SUCCESS]` messages
   - Configuration space obstacle counts increase with robot radius
   - Map dimensions match input specifications (6.80 x 9.93)
   - Both environment and configuration maps are generated

   **Example Output Interpretation:**
   ```
   Robot radius 0.100000m: 364587 obstacle pixels
   Robot radius 0.300000m: 444107 obstacle pixels  
   Robot radius 0.500000m: 518938 obstacle pixels
   ```
   This progression confirms correct obstacle expansion behavior.

   ``` bash
    br@br:~/workspace/pepper_rob_ws$ roslaunch unit_tests mapGenerationLaunchTestHarness.launch 
    ... logging to /home/br/.ros/log/a378e318-4209-11f0-8fae-9b78bf47ac01/roslaunch-br-80577.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://172.29.111.239:43271/

    SUMMARY
    ========

    PARAMETERS
    * /mapGenerationTest/mapgen_package_path: /home/br/workspac...
    * /mapGenerationTest/test_package_path: /home/br/workspac...
    * /mapgen_data_dir: /home/br/workspac...
    * /rosdistro: noetic
    * /rosversion: 1.17.0
    * /test_data_dir: /home/br/workspac...
    * /test_timeout: 60.0

    NODES
      /
        mapGenerationTest (unit_tests/mapGenerationTest)

    ROS_MASTER_URI=http://172.29.111.239:11311

    process[mapGenerationTest-1]: started with pid [80602]
    [ INFO] [1749131075.531971134]: mapGenerationTest: v1.0

                                    This project is funded by the African Engineering and Technology Network
                                    (Afretec) Inclusive Digital Transformation Research Grant Programme.
                                    Website: www.cssr4africa.org
                                    This program comes with ABSOLUTELY NO WARRANTY

    [ INFO] [1749131075.532004911]: mapGenerationTest: start-up.

    [2025-06-05 15:44:35] mapGenerationTest: =======================================================================
    [2025-06-05 15:44:35] mapGenerationTest: === New Map Generation Test Run Started at 2025-06-05 15:44:35 ===
    [2025-06-05 15:44:35] mapGenerationTest: =======================================================================
    [2025-06-05 15:44:35] mapGenerationTest: Testing Actual Map Generation Node: STARTED
    [2025-06-05 15:44:35] mapGenerationTest: -----------------------------------------------------------------------
    [2025-06-05 15:44:35] mapGenerationTest: Test Case: MapGenerationTest.BasicMapGeneration
    [2025-06-05 15:44:35] mapGenerationTest: Loaded radius1: 0.100000m
    [2025-06-05 15:44:35] mapGenerationTest: Loaded radius2: 0.200000m
    [2025-06-05 15:44:35] mapGenerationTest: Loaded radius3: 0.300000m
    [2025-06-05 15:44:35] mapGenerationTest: Loaded radius4: 0.500000m
    [2025-06-05 15:44:35] mapGenerationTest: Loaded radius5: 0.800000m
    [2025-06-05 15:44:35] mapGenerationTest: Testing basic map generation with actual mapGeneration node...
    [2025-06-05 15:44:35] mapGenerationTest: Launching actual mapGeneration node...
    [2025-06-05 15:45:05] mapGenerationTest: [SUCCESS] mapGeneration node completed execution
    [2025-06-05 15:45:06] mapGenerationTest: [SUCCESS] Both output maps were created successfully
    [2025-06-05 15:45:06] mapGenerationTest: Validating map dimensions...
    [2025-06-05 15:45:06] mapGenerationTest: Maps have correct dimensions: 680x993
    [2025-06-05 15:45:06] mapGenerationTest: Environment map obstacles: 306011
    [2025-06-05 15:45:06] mapGenerationTest: Configuration space obstacles: 444107
    [2025-06-05 15:45:06] mapGenerationTest: [SUCCESS] Configuration space correctly has more obstacles than environment map
    [2025-06-05 15:45:06] mapGenerationTest: Basic map generation test: PASSED
    [2025-06-05 15:45:06] mapGenerationTest: -----------------------------------------------------------------------
    [2025-06-05 15:45:06] mapGenerationTest: Test Case: MapGenerationTest.DifferentRobotRadii
    [2025-06-05 15:45:06] mapGenerationTest: Loaded radius1: 0.100000m
    [2025-06-05 15:45:06] mapGenerationTest: Loaded radius2: 0.200000m
    [2025-06-05 15:45:06] mapGenerationTest: Loaded radius3: 0.300000m
    [2025-06-05 15:45:06] mapGenerationTest: Loaded radius4: 0.500000m
    [2025-06-05 15:45:06] mapGenerationTest: Loaded radius5: 0.800000m
    [2025-06-05 15:45:06] mapGenerationTest: Testing different robot radii configurations...
    [2025-06-05 15:45:06] mapGenerationTest: Testing with robot radius: 0.100000m
    [2025-06-05 15:45:06] mapGenerationTest: Launching actual mapGeneration node...
    [2025-06-05 15:45:06] mapGenerationTest: [SUCCESS] mapGeneration node completed execution
    [2025-06-05 15:45:07] mapGenerationTest: [SUCCESS] Both output maps were created successfully
    [2025-06-05 15:45:07] mapGenerationTest: Validating map dimensions...
    [2025-06-05 15:45:07] mapGenerationTest: Maps have correct dimensions: 680x993
    [2025-06-05 15:45:07] mapGenerationTest: Environment map obstacles: 306011
    [2025-06-05 15:45:07] mapGenerationTest: Configuration space obstacles: 364587
    [2025-06-05 15:45:07] mapGenerationTest: [SUCCESS] Configuration space correctly has more obstacles than environment map
    [2025-06-05 15:45:07] mapGenerationTest: Robot radius 0.100000m: 364587 obstacle pixels
    [2025-06-05 15:45:07] mapGenerationTest: Testing with robot radius: 0.300000m
    [2025-06-05 15:45:07] mapGenerationTest: Launching actual mapGeneration node...
    [2025-06-05 15:45:07] mapGenerationTest: [SUCCESS] mapGeneration node completed execution
    [2025-06-05 15:45:08] mapGenerationTest: [SUCCESS] Both output maps were created successfully
    [2025-06-05 15:45:08] mapGenerationTest: Validating map dimensions...
    [2025-06-05 15:45:08] mapGenerationTest: Maps have correct dimensions: 680x993
    [2025-06-05 15:45:08] mapGenerationTest: Environment map obstacles: 306011
    [2025-06-05 15:45:08] mapGenerationTest: Configuration space obstacles: 444107
    [2025-06-05 15:45:08] mapGenerationTest: [SUCCESS] Configuration space correctly has more obstacles than environment map
    [2025-06-05 15:45:08] mapGenerationTest: Robot radius 0.300000m: 444107 obstacle pixels
    [2025-06-05 15:45:08] mapGenerationTest: Testing with robot radius: 0.500000m
    [2025-06-05 15:45:08] mapGenerationTest: Launching actual mapGeneration node...
    [2025-06-05 15:45:09] mapGenerationTest: [SUCCESS] mapGeneration node completed execution
    [2025-06-05 15:45:09] mapGenerationTest: [SUCCESS] Both output maps were created successfully
    [2025-06-05 15:45:09] mapGenerationTest: Validating map dimensions...
    [2025-06-05 15:45:09] mapGenerationTest: Maps have correct dimensions: 680x993
    [2025-06-05 15:45:09] mapGenerationTest: Environment map obstacles: 306011
    [2025-06-05 15:45:09] mapGenerationTest: Configuration space obstacles: 518938
    [2025-06-05 15:45:09] mapGenerationTest: [SUCCESS] Configuration space correctly has more obstacles than environment map
    [2025-06-05 15:45:09] mapGenerationTest: Robot radius 0.500000m: 518938 obstacle pixels
    [2025-06-05 15:45:09] mapGenerationTest: Different robot radii test: PASSED
    [2025-06-05 15:45:09] mapGenerationTest: -----------------------------------------------------------------------
    [2025-06-05 15:45:09] mapGenerationTest: Test Case: MapGenerationTest.VerboseModeTest
    [2025-06-05 15:45:09] mapGenerationTest: Loaded radius1: 0.100000m
    [2025-06-05 15:45:09] mapGenerationTest: Loaded radius2: 0.200000m
    [2025-06-05 15:45:09] mapGenerationTest: Loaded radius3: 0.300000m
    [2025-06-05 15:45:09] mapGenerationTest: Loaded radius4: 0.500000m
    [2025-06-05 15:45:09] mapGenerationTest: Loaded radius5: 0.800000m
    [2025-06-05 15:45:09] mapGenerationTest: Testing verbose mode configuration...
    [2025-06-05 15:45:09] mapGenerationTest: Launching actual mapGeneration node...
    [2025-06-05 15:45:39] mapGenerationTest: [SUCCESS] mapGeneration node completed execution
    [2025-06-05 15:45:40] mapGenerationTest: [SUCCESS] Both output maps were created successfully
    [2025-06-05 15:45:40] mapGenerationTest: Validating map dimensions...
    [2025-06-05 15:45:40] mapGenerationTest: Maps have correct dimensions: 680x993
    [2025-06-05 15:45:40] mapGenerationTest: Environment map obstacles: 306011
    [2025-06-05 15:45:40] mapGenerationTest: Configuration space obstacles: 444107
    [2025-06-05 15:45:40] mapGenerationTest: [SUCCESS] Configuration space correctly has more obstacles than environment map
    [2025-06-05 15:45:40] mapGenerationTest: Verbose mode configuration test: PASSED
    [2025-06-05 15:45:40] mapGenerationTest: -----------------------------------------------------------------------
    [2025-06-05 15:45:40] mapGenerationTest: =======================================================================
    [2025-06-05 15:45:40] mapGenerationTest: === Map Generation Test Run (CAD Mode) Completed at 2025-06-05 15:45:40 ===
    [2025-06-05 15:45:40] mapGenerationTest: === All tests PASSED ===
    [2025-06-05 15:45:40] mapGenerationTest: =======================================================================
    ================================================================================REQUIRED process [mapGenerationTest-1] has died!
    process has finished cleanly
    log file: /home/br/.ros/log/a378e318-4209-11f0-8fae-9b78bf47ac01/mapGenerationTest-1*.log
    Initiating shutdown!
    ================================================================================
    [mapGenerationTest-1] killing on exit
    shutting down processing monitor...
    ... shutting down processing monitor complete
    done
   ```

## Troubleshooting

### Common Test Failures and Solutions:

**File Not Found Errors:**
- Ensure all data files exist in `unit_tests/mapGenerationTest/data/`
- Verify obstacle file names match those specified in input files

**Dimension Mismatch:**
- Check that `mapGenerationInput.dat` specifies correct width and height values
- Verify obstacle coordinates fall within map boundaries

**Configuration Space Validation Failure:**
- Ensure robot radius values are positive and reasonable (0.1m - 1.0m)
- Check that obstacles are not too complex for the specified map size

**ROS Communication Issues:**
- Verify roscore is running before starting tests
- Ensure ROS environment variables are set correctly

<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
     <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
     <span style="color: #cccccc;">To fully understand the testing procedures, configuration values, and the overall functionality of the mapGenerationTest node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.3.pdf" style="color: #66b3ff;">D5.5.3 Environment Map Generation</a>. This document provides comprehensive explanations and step-by-step instructions essential for effective testing and troubleshooting.</span>
</div>

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:bgirmash@andrew.cmu.edu">bgirmash@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>


## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date: 2025-06-05