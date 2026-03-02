/* gestureExecutionTestImplementationcpp
*
* Author: Adedayo Akinade
* Date: January 10, 2025
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#include "gestureExecutionTest/gestureExecutionTestInterface.h"

// The test report file variables
std::string test_report_file = "gestureExecutionTestOutput.dat";       // data filename
std::string test_report_path;                                          // data path
std::string test_report_path_and_file;                                 // data path and filename

// The gesture execution test input file variables
bool run_iconic_gestures_test = false;                                  // Run iconic gestures test
bool run_deictic_gestures_test = false;                                 // Run deictic gestures test
bool run_bow_gestures_test = false;                                     // Run bow gestures test
bool run_nod_gestures_test = false;                                     // Run nod gestures test
bool run_symbolic_gestures_test = false;                                // Run symbolic gestures test

std::string node_name;                                                 // The name of the ROS node

ros::Publisher speech_pub;                                              // Publisher for speech commands
std_msgs::String speech_msg;                                            // Message for speech commands

/*  
 *  Function to read the gesture execution test input file
 *
 *  The function reads the gesture execution test input file and sets the variables for the gesture execution tests
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 *      topics_filename: the topics filename
 *      debug: the debug mode
 * 
 *  @return
 *      0 if the gesture execution test input file is read successfully
 *      1 if the gesture execution test input file is not read successfully
 */
int read_gesture_execution_test_configuration(string* platform, bool* debug) {
    std::string data_directory;                                      // data directory
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_directory = "/gestureExecutionTest/config/";
    data_path += data_directory;
    data_path_and_file = data_path;
    data_path_and_file += "gestureExecutionTestConfiguration.ini";

    // Open input file
    std::ifstream input_file(data_path_and_file.c_str());
    if (!input_file.is_open()){
        ROS_ERROR("%s: Failed to open the gesture execution test configuration file '%s'", node_name.c_str(), data_path_and_file.c_str());
        return 1;
    }

    else{
        std::string input_line_read;  // variable to read the line in the file
    
        std::string param_key, param_value;

        // Set the platform value to robot by default
        *platform = "robot";

        // Get key-value pairs from the input file
        while(std::getline(input_file, input_line_read)){
            std::istringstream input_line_stream(input_line_read);
            input_line_stream >> param_key >> param_value;
            trim(param_key);
            trim(param_value);

            // convert the key and value to lower case
            boost::algorithm::to_lower(param_value);
            
            // Set the platform value -- Removed from configuration file, set to robot by default but can be added if necessrary
            if(param_key == "platform"){
                *platform = param_value;
                if(*platform != "robot" && *platform != "simulator"){
                    ROS_ERROR("%s: incorrect platform value in the gesture execution test configuration file '%s'", node_name.c_str(), data_path_and_file.c_str());
                    return 1;
                }
            }
            else if(param_key == "iconic"){
                if(param_value == "true"){
                    run_iconic_gestures_test = true;
                }
                else{
                    run_iconic_gestures_test = false;
                }
            }
            else if((param_key == "deictic") || (param_key == "diectic")){
                if(param_value == "true"){
                    run_deictic_gestures_test = true;
                }
                else{
                    run_deictic_gestures_test = false;
                }
            }
            else if(param_key == "bow"){
                if(param_value == "true"){
                    run_bow_gestures_test = true;
                }
                else{
                    run_bow_gestures_test = false;
                }
            }
            else if(param_key == "nod"){
                if(param_value == "true"){
                    run_nod_gestures_test = true;
                }
                else{
                    run_nod_gestures_test = false;
                }
            }
            else if(param_key == "symbolic"){
                if(param_value == "true"){
                    run_symbolic_gestures_test = true;
                }
                else{
                    run_symbolic_gestures_test = false;
                }
            }
            else if(param_key == "verboseMode"){
                if(param_value == "true"){
                    *debug = true;
                }
                else{
                    *debug = false;
                }
            }
        }
        input_file.close();

        if(*platform == "robot"){
            ROS_INFO("%s: running on the robot platform...", node_name.c_str());
        }
        else if(*platform == "simulator"){
            ROS_INFO("%s: running on the simulator platform...", node_name.c_str());
        }
    }    
    return 0;
}

/*  
*   Function to write a string to a file
 *
 *  The function writes a string to a file
 *  
 *  @param:
 *      filename: the name of the file to write to
 *      directory: the directory where the file is located in the project workspace
 *      content: the content to write to the file
 *      separator: the separator to use in the file
 * 
 *  @return
 *      0 if the file is written successfully
 *      1 if the file is not written successfully
 */
int write_string_to_file(string filename, std::string directory, std::vector<std::vector<std::string>> content, std::string separator){
    std::string data_directory;                                      // data directory
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

     // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    // data_directory = "/" + directory + "/";
    data_path += "/gestureExecution";
    data_path += directory;
    data_path_and_file = data_path;
    data_path_and_file += filename;

    std::ofstream data_file(data_path_and_file, std::ios::out | std::ios::trunc); // Clear the file
    if (data_file.is_open()) {
        for (int i = 0; i < content.size(); i++) {
            for (int j = 0; j < content[i].size(); j++) {
                data_file << content[i][j] << separator;
            }
            data_file << "\n";
        }
        data_file.close();
        return 0;
    } else {
        ROS_ERROR("%s: Failed to open file '%s'", node_name.c_str(), data_path_and_file.c_str());
        return 1;
    }
}

/*  
 *  Function to delete a file
 *
 *  The function deletes a file
 *  
 *  @param:
 *      filename: the name of the file to delete
 *      directory: the directory where the file is located in the project workspace
 * 
 *  @return
 *      0 if the file is deleted successfully
 *      1 if the file is not deleted successfully
 */
int delete_file(string filename, std::string directory){
    std::string data_directory;                                      // data directory
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

     // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(CSSR_SYSTEM_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    // data_directory = "/" + directory + "/";
    data_path += "/gestureExecution";
    data_path += directory;
    data_path_and_file = data_path;
    data_path_and_file += filename;

    if (remove(data_path_and_file.c_str())) {
        ROS_ERROR("%s: Failed to delete file '%s'", node_name.c_str(), data_path_and_file.c_str());
        return 1;
    }
    return 0;
}

/*  
 *  Function to write the gesture execution configuration file
 *
 *  The function writes the gesture execution configuration file
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 *      interpolation: the interpolation type to use in the gesture execution tests
 *      gesture_descriptors: the gesture descriptors configuration file
 *      simulator_topics_filename: the simulator topics configuration file
 *      robot_topics_filename: the robot topics configuration file
 *      verbose_mode_input: the verbose mode input
 * 
 *  @return
 *      0 if the file is written successfully
 *      1 if the file is not written successfully
 */
int write_configuration_file(std::string platform, std::string interpolation, std::string gesture_descriptors, std::string simulator_topics_filename, std::string robot_topics_filename, std::string verbose_mode_input) {
    // Set the separator and data directory
    std::string separator = "\t\t\t";
    std::string data_directory = "/config/";

    // Set the contents of the gesture execution configuration file
    std::vector<std::vector<std::string>> configuration_content = {
        // {"platform\t\t", platform},                  // Uncommment this if you now have support for a simulator platform
        {"interpolation\t", interpolation},
        {"gestureDescriptors", gesture_descriptors},
        {"simulatorTopics\t", simulator_topics_filename},
        {"robotTopics\t\t", robot_topics_filename},
        {"verboseMode\t\t", verbose_mode_input}
    };

    // Write the gesture execution configuration file
    if(write_string_to_file("gestureExecutionConfiguration.ini", data_directory, configuration_content, separator) != 0){
        return 1;
    }
    return 0;
}

/*  Function to invoke the gesture execution service and return the response from the service
 *  
 * @param:
 *     cmd: the service command to run
 * 
 * @return
 *    result: the response from the service
 */
std::string invoke_service(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

/* 
 *   Function to verify if a topic is available
 * @param:
 *   topic_name: string to store the topic name
 * 
 * @return:
 *  boolean indicating if the topic is available
 */
bool is_topic_available(std::string topic_name){
    bool topic_available = false;                                                   // boolean to store if the topic is available
    ros::master::V_TopicInfo master_topics;                                         // vector to store the topics
    ros::master::getTopics(master_topics);                                          // get the topics

    // Iterate through the topics to check if the topic is available
    for (const auto& topic : master_topics){
        if (topic.name == topic_name){                                              // if the topic is found
            topic_available = true;                                                 // set the topic as available
            break;
        }
    }

    return topic_available;                                                         // return the topic availability
}

/* 
 *   Function to verify if a service is available
 * @param:
 *   service_name: string to store the service name
 * 
 * @return:
 *  boolean indicating if the service is available
 */
bool is_service_available(const std::string& service_name){
    if(ros::service::exists(service_name, false)){                                  // check if the service exists
        return true;                                                                // return true if the service exists
    }
    return false;                                                                   // return false if the service does not exist
}

/*  
 *   Function to handle the shutdown signal
 *   Cancels all active goals and shuts down ROS
 *  @param:
 *      sig: the signal
 *  @return:
 *      None
 */
void shut_down_handler(int sig){
    printf("\n");
    ROS_WARN("%s: shutting down...", node_name.c_str());

    // Shutdown the node
    ROS_ERROR("%s: terminated.", node_name.c_str());
    ros::shutdown();
}


/*  
 *  Function to run the iconic gestures test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(GestureExecutionUnitTest, TestIconicGestures) {
    // Skip this test if the iconic gestures test is not enabled
    if(!run_iconic_gestures_test){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running the ICONIC GESTURES test...", node_name.c_str());

    // Publish the speech command
    speech_msg.data = "ICONIC           GESTURES            TEST        ";
    speech_pub.publish(speech_msg);
    ros::Duration(3).sleep();

    // Set the iconic gestures IDs and names
    std::vector<string> iconic_gesture_IDs = {"01", "02", "03", "04", "05"};
    std::vector<string> iconic_gesture_names = {"Welcome Gesture", "Welcome Gesture", "Wave Right Hand", "Shake Gesture", "Shake Gesture"};
    
    // Set the iconic gestures identifiers
    std::vector<std::vector<string>> iconic_gesture_identifier;
    std::vector<std::vector<vector<string>>> welcome_gesture_identifier;
    std::vector<std::vector<vector<string>>> wave_gesture_identifier;
    std::vector<std::vector<vector<string>>> shake_gesture_identifier;
    std::vector<std::vector<vector<vector<string>>>> iconic_gestures_identifiers;

    // Set the gesture descriptor configuration file content
    std::vector<std::vector<std::string>> gesture_descriptor_config_content = {
        {"01", "LArm", "lArmWelcomeGestureDescriptorsTest.dat"},
        {"02", "RArm", "rArmWelcomeGestureDescriptorsTest.dat"},
        {"03", "RArm", "waveGestureDescriptorsTest.dat"},
        {"04", "LArm", "lArmShakeGestureDescriptors.dat"},
        {"05", "RArm", "rArmShakeGestureDescriptors.dat"}
    };
    std::string data_directory = "/data/";
    std::string separator = "\t";

    // Write the gesture descriptor configuration file
    std::string gesture_descriptors_configuration_filename = "gestureDescriptorsTest.dat";
    write_string_to_file(gesture_descriptors_configuration_filename, data_directory, gesture_descriptor_config_content, separator);

    // Set the joint names for the iconic gestures in a left arm welcome gesture
    std::vector<string> left_arm_welcome_gesture_joint_names = {
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"
    };
    iconic_gesture_identifier.push_back(left_arm_welcome_gesture_joint_names);

    // Set the joint angles for the iconic gestures in a left arm welcome gesture
    std::vector<std::string> left_arm_welcome_gesture_waypoints = {
        {"1.7625 0.09970 -1.7150 -0.1334 0.06592"},
        {"1.047197 0.2618 -1.5708 -0.0087 -1.047198"},
        {"1.7625 0.09970 -1.7150 -0.1334 0.06592"}
    };
    iconic_gesture_identifier.push_back(left_arm_welcome_gesture_waypoints);

    // Save the iconic gesture identifier into the welcome gesture identifier
    welcome_gesture_identifier.push_back(iconic_gesture_identifier);
    iconic_gesture_identifier.clear();

    // 
    std::string left_arm_welcome_gestures_joint_angles = "";
    for (int i = 0; i < left_arm_welcome_gesture_waypoints.size(); i++) {
        left_arm_welcome_gestures_joint_angles += left_arm_welcome_gesture_waypoints[i];
        if (i < left_arm_welcome_gesture_waypoints.size() - 1) {
            left_arm_welcome_gestures_joint_angles += ";";
        }
    }

    // Write the left arm welcome gesture descriptor file   
    std::string left_arm_welcome_gesture_filename = "lArmWelcomeGestureDescriptorsTest.dat";
    std::vector<std::vector<std::string>> left_arm_welcome_gesture_contents = {
        {"type\t", "iconic"},
        {"ID\t\t", "01"},
        {"wayPoints", "3"},
        {"jointAngles", left_arm_welcome_gestures_joint_angles}
    };
    separator = "\t\t\t\t";
    write_string_to_file(left_arm_welcome_gesture_filename, data_directory, left_arm_welcome_gesture_contents, separator);

    // Set the joint names for the iconic gestures in a right arm welcome gesture
    std::vector<string> right_arm_welcome_gesture_joint_names = {
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"
    };
    iconic_gesture_identifier.push_back(right_arm_welcome_gesture_joint_names);

    // Set the joint angles for the iconic gestures in a right arm welcome gesture
    std::vector<std::string> right_arm_welcome_gesture_waypoints = {
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"},
        {"1.047197 -0.2618 1.5708 0.0087 1.047198"},
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"}
    };
    iconic_gesture_identifier.push_back(right_arm_welcome_gesture_waypoints);

    // Save the iconic gesture identifier into the welcome gesture identifier
    welcome_gesture_identifier.push_back(iconic_gesture_identifier);
    iconic_gesture_identifier.clear();


    std::string right_arm_welcome_gestures_joint_angles = "";
    for (int i = 0; i < right_arm_welcome_gesture_waypoints.size(); i++) {
        right_arm_welcome_gestures_joint_angles += right_arm_welcome_gesture_waypoints[i];
        if (i < right_arm_welcome_gesture_waypoints.size() - 1) {
            right_arm_welcome_gestures_joint_angles += ";";
        }
    }

    // Write the right arm welcome gesture descriptor file
    std::string right_arm_welcome_gesture_filename = "rArmWelcomeGestureDescriptorsTest.dat";
    std::vector<std::vector<std::string>> right_arm_welcome_gesture_contents = {
        {"type\t", "iconic"},
        {"ID\t\t", "02"},
        {"wayPoints", "3"},
        {"jointAngles", right_arm_welcome_gestures_joint_angles}
    };
    separator = "\t\t\t\t";
    write_string_to_file(right_arm_welcome_gesture_filename, data_directory, right_arm_welcome_gesture_contents, separator);

    // Set the joint names for the iconic gestures in a wave right hand gesture
    std::vector<string> wave_gesture_joint_names = {
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"
    };
    iconic_gesture_identifier.push_back(wave_gesture_joint_names);

    // Set the joint angles for the iconic gestures in a wave right hand gesture
    std::vector<std::string> wave_gesture_waypoints = {
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 0.5236 0.0"},
        {"0.04139658848768486 -0.7725247646974353 1.4899726916768 1.308996 0.0"},
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"}
    };
    iconic_gesture_identifier.push_back(wave_gesture_waypoints);

    // Save the iconic gesture identifier into the wave gesture identifier
    wave_gesture_identifier.push_back(iconic_gesture_identifier);
    iconic_gesture_identifier.clear();

    std::string wave_gestures_joint_angles = "";
    for (int i = 0; i < wave_gesture_waypoints.size(); i++) {
        wave_gestures_joint_angles += wave_gesture_waypoints[i];
        if (i < wave_gesture_waypoints.size() - 1) {
            wave_gestures_joint_angles += ";";
        }
    }

    // Write the wave gesture descriptor file
    std::string wave_gesture_filename = "waveGestureDescriptorsTest.dat";
    std::vector<std::vector<std::string>> wave_gesture_contents = {
        {"type\t", "iconic"},
        {"ID\t\t", "03"},
        {"wayPoints", "10"},
        {"jointAngles", wave_gestures_joint_angles}
    };
    separator = "\t\t\t\t";
    write_string_to_file(wave_gesture_filename, data_directory, wave_gesture_contents, separator);



    // -----------------------------------

    // Set the joint names for the iconic gestures in a left arm shake gesture
    std::vector<string> left_arm_shake_gesture_joint_names = {
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"
    };
    iconic_gesture_identifier.push_back(left_arm_shake_gesture_joint_names);

    // Set the joint angles for the iconic gestures in a left arm shake gesture
    std::vector<std::string> left_arm_shake_gesture_waypoints = {
        {"1.7625 0.09970 -1.7150 -0.1334 0.06592"},
        {"0.358 0.15 0.066 -1.333 -1.8"},
        {"0.358 0.15 0.066 -1.333 -1.8"},
        {"1.7625 0.09970 -1.7150 -0.1334 0.06592"}
    };
    iconic_gesture_identifier.push_back(left_arm_shake_gesture_waypoints);

    // Save the iconic gesture identifier into the shake gesture identifier
    shake_gesture_identifier.push_back(iconic_gesture_identifier);
    iconic_gesture_identifier.clear();

    // 
    std::string left_arm_shake_gestures_joint_angles = "";
    for (int i = 0; i < left_arm_shake_gesture_waypoints.size(); i++) {
        left_arm_shake_gestures_joint_angles += left_arm_shake_gesture_waypoints[i];
        if (i < left_arm_shake_gesture_waypoints.size() - 1) {
            left_arm_shake_gestures_joint_angles += ";";
        }
    }

    // Write the left arm shake gesture descriptor file   
    std::string left_arm_shake_gesture_filename = "lArmShakeGestureDescriptorsTest.dat";
    std::vector<std::vector<std::string>> left_arm_shake_gesture_contents = {
        {"type\t", "iconic"},
        {"ID\t\t", "04"},
        {"wayPoints", "2"},
        {"jointAngles", left_arm_shake_gestures_joint_angles}
    };
    separator = "\t\t\t\t";
    write_string_to_file(left_arm_shake_gesture_filename, data_directory, left_arm_shake_gesture_contents, separator);



    // ------------------------------------



    // -----------------------------------

    // Set the joint names for the iconic gestures in a right arm shake gesture
    std::vector<string> right_arm_shake_gesture_joint_names = {
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"
    };
    iconic_gesture_identifier.push_back(right_arm_shake_gesture_joint_names);

    // Set the joint angles for the iconic gestures in a left arm shake gesture
    std::vector<std::string> right_arm_shake_gesture_waypoints = {
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"},
        {"0.183 -0.02 -0.082 0.5 1.634"},
        {"0.183 -0.02 -0.082 0.5 1.634"},
        {"1.7410 -0.09664 1.6981 0.09664 -0.05679"}
    };
    iconic_gesture_identifier.push_back(right_arm_shake_gesture_waypoints);

    // Save the iconic gesture identifier into the shake gesture identifier
    shake_gesture_identifier.push_back(iconic_gesture_identifier);
    iconic_gesture_identifier.clear();

    // 
    std::string right_arm_shake_gestures_joint_angles = "";
    for (int i = 0; i < right_arm_shake_gesture_waypoints.size(); i++) {
        right_arm_shake_gestures_joint_angles += right_arm_shake_gesture_waypoints[i];
        if (i < right_arm_shake_gesture_waypoints.size() - 1) {
            right_arm_shake_gestures_joint_angles += ";";
        }
    }

    // Write the left arm shake gesture descriptor file   
    std::string right_arm_shake_gesture_filename = "rArmShakeGestureDescriptorsTest.dat";
    std::vector<std::vector<std::string>> right_arm_shake_gesture_contents = {
        {"type\t", "iconic"},
        {"ID\t\t", "05"},
        {"wayPoints", "2"},
        {"jointAngles", right_arm_shake_gestures_joint_angles}
    };
    separator = "\t\t\t\t";
    write_string_to_file(right_arm_shake_gesture_filename, data_directory, right_arm_shake_gesture_contents, separator);



    // ------------------------------------





    // Save the iconic gestures identifiers
    iconic_gestures_identifiers.push_back(welcome_gesture_identifier);
    iconic_gestures_identifiers.push_back(welcome_gesture_identifier);
    iconic_gestures_identifiers.push_back(wave_gesture_identifier);
    iconic_gestures_identifiers.push_back(shake_gesture_identifier);
    iconic_gestures_identifiers.push_back(shake_gesture_identifier);

    std::vector<string> iconic_gesture_speech_names = {"Welcome                     Gesture", "Welcome                      Gesture", \
                                                        "Wave                   Right                   Hand", "Shake                   Gesture", \
                                                            "Shake                  Gesture"};
    // Execute iconic gestures
    for (int i = 0; i < iconic_gesture_IDs.size(); i++) {
        // Extract the iconic gesture details
        string gesture_ID = iconic_gesture_IDs[i];
        string gesture_name = iconic_gesture_names[i];
        vector<vector<vector<string>>> gesture_identifier = iconic_gestures_identifiers[i];

        // Publish the speech command
        speech_msg.data = "ID           " + gesture_ID + "                      " + iconic_gesture_speech_names[i] + "                      " ;
        speech_pub.publish(speech_msg);
        
        // Execute iconic gesture by calling the gesture execution service with a system call and save the response from the service
        std::string output; 
        string service_call = "rosservice call /gestureExecution/perform_gesture -- iconic " + gesture_ID + " 3000 45 -1.2 3 0.82";
        output = invoke_service(service_call.c_str());

        bool test_result = false;                   // Status of the test
        // Parse the integer from the response from the service
        int response;
        if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
            if(response == 1){            // Check if the response is 1 and set the test result to true
                test_result = true;
            }
            else{                         // Set the test result to false
                test_result = false;
            }
        } else {
            FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
        }

        // Log the result of the test
        std::string test_name = "Iconic Gesture " + gesture_ID + " : " + gesture_name;
        logIconicTestResult(test_name, gesture_identifier, test_result);

        ros::Duration(2).sleep();
    }

    // Delete the descriptor files
    delete_file(gesture_descriptors_configuration_filename, data_directory);
    delete_file(left_arm_welcome_gesture_filename, data_directory);
    delete_file(right_arm_welcome_gesture_filename, data_directory);
    delete_file(wave_gesture_filename, data_directory);
    delete_file(left_arm_shake_gesture_filename, data_directory);
    delete_file(right_arm_shake_gesture_filename, data_directory);
}

/*  
 *  Function to run the deictic gestures test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(GestureExecutionUnitTest, TestDiecticGestures) {
    // Skip this test if the deictic gestures test is not enabled
    if(!run_deictic_gestures_test){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running the DEICTIC GESTURES test...", node_name.c_str());

    // Publish the speech command
    speech_msg.data = "DEICTIC           GESTURES            TEST        ";
    speech_pub.publish(speech_msg);
    ros::Duration(3).sleep();

    // Set the deictic gestures point coordinates
    std::vector<vector<string>> point_coordinates = {
        {"0.6", "5.1", "0.82"},
        {"6.8", "4.8", "0.82"},
        {"5.0", "1.8", "0.82"},
        {"6.8", "9.6", "0.82"}
    };

    for (int i = 0; i < point_coordinates.size(); i++) {
        // Extract the point coordinates
        string point_x = point_coordinates[i][0];
        string point_y = point_coordinates[i][1];
        string point_z = point_coordinates[i][2];

        // Publish the speech command
        speech_msg.data = "         X                                      " + point_x + "                                                                                                                  " + \
                             "          Y                                                            " + point_y + "                                                                                               " + \
                             "          Z                                                                    " + point_z + "                                                          ";
        speech_pub.publish(speech_msg);     

        // Execute diectic gesture by calling the gesture execution service with a system call and save the response from the service
        std::string output; 
        string service_call = "rosservice call /gestureExecution/perform_gesture -- deictic 02 3000 45 " + point_x + " " + point_y + " " + point_z;
        output = invoke_service(service_call.c_str());
        ros::Duration(2).sleep();

        bool test_result = false;                   // Status of the test
        // Parse the integer from the response from the service
        int response;
        if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
            if(response == 1){            // Check if the response is 1 and set the test result to true
                test_result = true;
            }
            else{                         // Set the test result to false
                test_result = false;
            }
        } else {
            FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
        }

        // Log the result of the test
        std::string test_name = "Deictic Gesture \n\tTo\t\t\t  : " + point_x + " " + point_y + " " + point_z;
        logOtherTestResult(test_name, test_result);
    }
}

/*  
 *  Function to run the bow gestures test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(GestureExecutionUnitTest, TestBowGestures) {
    // Skip this test if the bow gestures test is not enabled
    if(!run_bow_gestures_test){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running the BOW GESTURES test...", node_name.c_str());

    // Publish the speech command
    speech_msg.data = "BOW           GESTURES            TEST        ";
    speech_pub.publish(speech_msg);
    ros::Duration(3).sleep();

    // Set the bow gestures angles
    std::vector<string> bow_angles = {"15", "30", "45"};

    for (int i = 0; i < bow_angles.size(); i++) {
        // Extract the bow angle
        string bow_angle = bow_angles[i];

        // Publish the speech command
        speech_msg.data = "                     Bow                                  angle                                      " + \
                            bow_angle + "                                                                                                                  " + \
                            "degrees                                                                                                                  ";            
        speech_pub.publish(speech_msg);   

        // Execute bow gesture by calling the gesture execution service with a system call and save the response from the service
        std::string output;
        string service_call = "rosservice call /gestureExecution/perform_gesture -- bow 02 2000 " + bow_angle + " 0 0 0";
        output = invoke_service(service_call.c_str());
        ros::Duration(2).sleep();

        bool test_result = false;                   // Status of the test
        // Parse the integer from the response from the service
        int response;
        if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
            if(response == 1){            // Check if the response is 1 and set the test result to true
                test_result = true;
            }
            else{                         // Set the test result to false
                test_result = false;
            }
        } else {
            FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
        }

        // Log the result of the test
        std::string test_name = "Bow Gesture \n\tDegree\t\t  : " + bow_angle;
        logOtherTestResult(test_name, test_result);
    }
}

/*  Function to run the nod gestures test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(GestureExecutionUnitTest, TestNodGestures) {
    // Skip this test if the nod gestures test is not enabled
    if(!run_nod_gestures_test){
        GTEST_SKIP();
    }

    ROS_INFO("%s: Running the NOD GESTURES test...", node_name.c_str());

    // Publish the speech command
    speech_msg.data = "NOD           GESTURES            TEST        ";
    speech_pub.publish(speech_msg);
    ros::Duration(3).sleep();

    // Set the nod gestures angles
    std::vector<string> nod_angles = {"15", "30", "45"};

    for (int i = 0; i < nod_angles.size(); i++) {
        // Extract the nod angle
        string nod_angle = nod_angles[i];

        // Publish the speech command
        speech_msg.data = "                     Nod                                  angle                                      " + \
                            nod_angle + "                                                                                                                  " + \
                            "degrees                                                                                                                  ";            
        speech_pub.publish(speech_msg);   

        // Execute nod gesture by calling the gesture execution service with a system call and save the response from the service
        std::string output;
        string service_call = "rosservice call /gestureExecution/perform_gesture -- nod 02 2000 " + nod_angle + " 0 0 0";
        output = invoke_service(service_call.c_str());
        ros::Duration(2).sleep();

        bool test_result = false;                   // Status of the test
        // Parse the integer from the response from the service
        int response;
        if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
            if(response == 1){            // Check if the response is 1 and set the test result to true
                test_result = true;
            }
            else{                         // Set the test result to false
                test_result = false;
            }
        } else {
            FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
        }

        // Log the result of the test
        std::string test_name = "Nod Gesture \n\tDegree\t\t  : " + nod_angle;
        logOtherTestResult(test_name, test_result);
    }
}