
<div align="center">
  <h1>Text-to-Speech Unit Tests</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides comprehensive unit tests for the `textToSpeech` node within the CSSR4Africa project (`cssr_system` package). The test suite validates multilingual text-to-speech functionality including English and Kinyarwanda language support, audio generation pipelines, robot communication pathways, and service interface robustness. The tests include application structure validation, integration testing with real audio output, and end-to-end system verification. The results ensure reliable TTS operation across different languages, platforms, and usage scenarios for Pepper robot applications.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the TTS system architecture and testing methodology. The deliverable report can be found in [D5.5.2.4 Text-to-Speech ](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf).

# Run the Text-to-Speech Unit Tests 
## Physical Robot 
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

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
         cd .. && source devel/setup.bash && catkin_make
       ```

3. **Setup TTS Environment:**

 
   ```bash
   # Activate the TTS virtual environment
   source ~/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_text_to_speech_env/bin/activate
   
   # Verify Python version and TTS package
   python --version 
   python -c "import TTS; print('TTS package available')"
   ```
       
4. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/text_to_speech/config/text_to_speech_configuration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `language` | Default language for TTS | `english` or `kinyarwanda` |
   | `verboseMode` | Enable detailed logging for testing | `True` or `False` |
   | `ip` | Pepper robot IP address | e.g., `172.29.111.240` |
   | `port` | Robot communication port | `9559` (default) |
   | `useCuda` | Enable GPU acceleration | `True` or `False` |


   - To execute tests on the physical platform, ensure the IP address matches your robot's configuration
   - Enable verbose mode for detailed test output and debugging information

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf" style="color: #66b3ff;">D5.5.2.4 Text-to-Speech </a>. Otherwise, the preferred values are the ones already set in the `text_to_speech_configuration.ini` file.</span>
  </div>

5. **Make test files executable:**

   ```bash
   cd $HOME/workspace/pepper_rob_ws/src/cssr4africa/unit_test
   chmod +x text_to_speech_test_application.py   
   ```

6. **Run the `textToSpeechTest` from the `unit_test` package**. 

    Follow below steps, run in different terminals.

    ### Option 1: Run Individual Test Suites

    -  Source the workspace in first terminal:
        ```bash
        cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot with TTS service:
        ```bash
        cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash

        roslaunch unit_tests text_to_speech_test_launch_robot.launch robot_ip:=<robot_ip> network_interface:=<network_interface>

        roslaunch unit_tests text_to_speech_test_launch_test_harness.launch run_tests:=false
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP address <code>robot_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>

    - Open new terminals to run individual test suites (ensure TTS service is running):
        ```bash
        # Source the virtual environment
        source $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs/cssr4africa_text_to_speech_env/bin/activate

        cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
          
        # Application Tests (No TTS service required)
        rosrun unit_test text_to_speech_test_application.py
        ```

    ### Option 2: Launch All Tests Simultaneously

    ```bash
    cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
    roslaunch unit_tests text_to_speech_test_launch_robot.launch robot_ip:=<robot_ip> network_interface:=<network_interface>
    roslaunch unit_tests text_to_speech_test_launch_test_harness.launch run_tests:=true
    ```


## Tests Executed
### Application Tests
The Application Tests validate the core TTS application structure and dependencies:
  - **Module Import Test**: Verifies TTS application module can be imported correctly from the cssr_system package
  - **Function Existence Test**: Confirms required functions (`handle_say_text`, `text_to_speech_node`) exist and are callable
  - **Dependencies Test**: Validates ROS dependencies, service message types, and required Python packages
  - **Path Configuration Test**: Tests the path setting mechanism for model files and configuration files
  - **Structure Validation Test**: Verifies overall application architecture and component integration

The robot/system is expected to have all dependencies properly configured and the TTS application structure correctly set up.

### Implementation Integration Tests
The Implementation Integration Tests verify real TTS functionality with actual robot interaction:
  - **English TTS Integration**: Tests English text-to-speech with real robot audio output and user verification
  - **Kinyarwanda TTS Integration**: Tests Kinyarwanda neural synthesis with custom model audio generation and playback
  - **Language Switching**: Validates seamless switching between different languages within the same session
  - **Edge Case Handling**: Tests empty messages, unsupported languages, long text processing, and boundary conditions
  - **Service Communication**: Verifies ROS service request/response handling and network communication protocols

The robot is expected to speak test messages clearly in both English and Kinyarwanda, requiring user confirmation of audio quality and language accuracy.


## Results
The results of the tests are displayed in real-time console output and logged appropriately. Test results include detailed pass/fail status, execution times, and diagnostic information for debugging.

### Physical Robot Results:
Console output is displayed in the terminal with colored status indicators for easy identification of test results:

```
============================================================
Text-to-Speech Implementation Integration Test Report
============================================================
Date: 2025-06-18 15:52:29

[2025-06-18 15:51:37] USER_INPUT: User started implementation testing
[2025-06-18 15:51:37] TEST_START: Starting English TTS Implementation Integration test
[2025-06-18 15:51:37] TEST_RESULT: Service call successful for message: 'world' in English
[2025-06-18 15:51:42] TEST_RESULT: TEST 1.1 - English first message: PASSED
[2025-06-18 15:51:43] TEST_RESULT: Service call successful for message: 'Hello' in English
[2025-06-18 15:51:46] TEST_RESULT: TEST 1.2 - English second message: PASSED
[2025-06-18 15:51:48] TEST_START: Starting Kinyarwanda TTS Implementation Integration test
[2025-06-18 15:51:52] TEST_RESULT: Service call successful for message: 'Muraho neza' in Kinyarwanda
[2025-06-18 15:51:55] TEST_RESULT: TEST 2 - Kinyarwanda TTS Integration: PASSED
[2025-06-18 15:51:57] TEST_START: Starting Language Switching Implementation test
[2025-06-18 15:52:14] TEST_RESULT: TEST 3 - Language Switching Implementation: PASSED
[2025-06-18 15:52:16] TEST_START: Starting Empty Message Implementation test
[2025-06-18 15:52:16] TEST_RESULT: TEST 4 - Empty Message Implementation: PASSED (processed successfully)
[2025-06-18 15:52:17] TEST_START: Starting Unsupported Language Implementation test
[2025-06-18 15:52:17] TEST_RESULT: TEST 5 - Unsupported Language Implementation: PASSED (correctly rejected)
[2025-06-18 15:52:18] TEST_START: Starting Long Message Implementation test
[2025-06-18 15:52:18] TEST_RESULT: Service call successful for long message (121 characters)
[2025-06-18 15:52:29] TEST_RESULT: TEST 6 - Long Message Implementation: PASSED
[2025-06-18 15:52:29] SUMMARY: FINAL TEST RESULTS:
[2025-06-18 15:52:29] SUMMARY:  English TTS Implementation Integration - PASSED
[2025-06-18 15:52:29] SUMMARY:  Kinyarwanda TTS Implementation Integration - PASSED
[2025-06-18 15:52:29] SUMMARY:  Language Switching Implementation - PASSED
[2025-06-18 15:52:29] SUMMARY:  Empty Message Implementation - PASSED
[2025-06-18 15:52:29] SUMMARY:  Unsupported Language Implementation - PASSED
[2025-06-18 15:52:29] SUMMARY:  Long Message Implementation - PASSED
[2025-06-18 15:52:29] SUMMARY: Total tests run: 6
[2025-06-18 15:52:29] SUMMARY: Passed: 6
[2025-06-18 15:52:29] SUMMARY: Failed: 0
[2025-06-18 15:52:29] FINAL_RESULT: FINAL RESULT: ALL IMPLEMENTATION TESTS PASSED

```


### Test Output Files:
- Real-time console output with colored status indicators for immediate feedback
- Detailed error messages 
- User interaction prompts and confirmations for integration tests requiring human verification

## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, test methodology, debugging processes, and the overall functionality of the textToSpeechTest suite, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.2.4.pdf" style="color: #66b3ff;">D5.5.2.4 Text-to-Speech.pdf</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective testing, troubleshooting, and system validation.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:muhirwarichard1@gmail.com">muhirwarichard1@gmail.com</a>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

**Authors:** Muhirwa Richard, CSSR4Africa Consortium  
**Date:** May 2025