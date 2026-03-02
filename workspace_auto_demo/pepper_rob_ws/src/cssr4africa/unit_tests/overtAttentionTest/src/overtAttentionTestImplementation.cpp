/* overtAttentionTestImplementation.cpp
*
* Author: Muhammed Danso and Adedayo Akinade
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

#include "overtAttentionTest/overtAttentionTestInterface.h"

std::string node_name; // The name of the node

// The test report file variables
std::string test_report_file = "overtAttentionTestOutput.dat";         // data filename
std::string test_report_path;                                          // data path
std::string test_report_path_and_file;                                 // data path and filename

bool mutual_gaze_detected = false;                                     // Stores the status of mutual gaze detection
int status                = 0;
ros::Publisher speech_pub;

bool test_scanning_mode;
bool test_social_mode;
bool test_seeking_mode;
bool test_disabled_mode;
bool test_location_mode;

/*  
 *  Function to read the overt attention test configuration file
 *
 *  The function reads the attention test configuration file and sets the variables for the attention tests
 *  
 *  @param:
 *      platform: the platform to run the gesture execution tests
 * 
 *  @return
 *      0 if the attention test configuration file is read successfully
 *      1 if the attention test configuration file is not read successfully
 */
int read_overt_attention_test_configuration(string* platform, bool* debug) {
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
    data_directory = "/overtAttentionTest/config/";
    data_path += data_directory;
    data_path_and_file = data_path;
    data_path_and_file += "overtAttentionTestConfiguration.ini";

    // Open input file
    std::ifstream input_file(data_path_and_file.c_str());
    if (!input_file.is_open()){
        ROS_ERROR("%s: unable to open the configuration file %s\n", node_name.c_str(), data_path_and_file.c_str());
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
                    ROS_ERROR("%s: incorrect platform value in the overt attention test configuration file '%s'", node_name.c_str(), data_path_and_file.c_str());
                    return 1;
                }
            }
            else if(param_key == "scanning"){
                if(param_value == "true"){
                    test_scanning_mode = true;
                }
                else{
                    test_scanning_mode = false;
                }
            }
            else if(param_key == "social"){
                if(param_value == "true"){
                    test_social_mode = true;
                }
                else{
                    test_social_mode = false;
                }
            }
            else if(param_key == "seeking"){
                if(param_value == "true"){
                    test_seeking_mode = true;
                }
                else{
                    test_seeking_mode = false;
                }
            }
            else if(param_key == "location"){
                if(param_value == "true"){
                    test_location_mode = true;
                }
                else{
                    test_location_mode = false;
                }
            }
            else if(param_key == "disabled"){
                if(param_value == "true"){
                    test_disabled_mode = true;
                }
                else{
                    test_disabled_mode = false;
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

        return 0;
    }    
}

/*  Function to invoke the overt attention mode service and return the response from the service
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
 *  Function to run the scanning mode test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(OvertAttentionUnitTest, TestScanningMode) {
    if(!test_scanning_mode){
        GTEST_SKIP();
    }

    // Make service call to change mode and save the response from the service
    std::string output; 
    string service_call = "rosservice call /overtAttention/set_mode scanning 0 0 0";
    output = invoke_service(service_call.c_str());

    bool test_result = false;                   // Status of the test
    // Parse the integer from the response from the service
    int response;
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SCANNING                    MODE                    ACTIVATED                   ";
            speech_pub.publish(message);

            ros::Duration(60).sleep();                  // put to sleep for a while
        }
        else{                         // Set the test result to false
            test_result = false;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SCANNING                    MODE                    NOT                 ACTIVATED                   ";
            speech_pub.publish(message);

            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }

    
    // Log the result of the test
    std::string test_name = "Scanning Mode : ";
    logTestResult(test_name, test_result);
    
}

/*  
 *  Function to run the social mode test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(OvertAttentionUnitTest, TestSocialMode) {
    if(!test_social_mode){
        GTEST_SKIP();
    }

    // Make service call to change mode and save the response from the service
    std::string output; 
    string service_call = "rosservice call /overtAttention/set_mode social 0 0 0";
    output = invoke_service(service_call.c_str());

    bool test_result = false;                   // Status of the test
    // Parse the integer from the response from the service
    int response;
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SOCIAL                    MODE                    ACTIVATED                   ";;
            speech_pub.publish(message);
            ros::Duration(60).sleep();                  // put to sleep for a while
    
        }
        else{                         // Set the test result to false
            test_result = false;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SOCIAL                    MODE                    NOT                   ACTIVATED                   ";;
            speech_pub.publish(message);
            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }

    
    
    // Log the result of the test
    std::string test_name = "Social Mode : ";
    logTestResult(test_name, test_result);
    
}

/*  
 *  Function to run the seeking mode test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(OvertAttentionUnitTest, TestSeekingMode) {
    if(!test_seeking_mode){
        GTEST_SKIP();
    }

    // Make service call to change mode and save the response from the service
    std::string output; 
    string service_call = "rosservice call /overtAttention/set_mode seeking 0 0 0";
    output = invoke_service(service_call.c_str());

    bool test_result = false;                   // Status of the test
    // Parse the integer from the response from the service
    int response;
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SEEKING                    MODE                    ACTIVATED                   ";;
            speech_pub.publish(message);
            ros::Duration(5).sleep();                  // put to sleep for a while
        }
        else{                         // Set the test result to false
            test_result = false;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    SEEKING                    MODE                    NOT                  ACTIVATED                   ";;
            speech_pub.publish(message);
            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }

    while (status == DETECTING_MUTUAL_GAZE || status == 0)
    {
        /* code */
        ros::spinOnce();
    }

    // Name the test
    std::string test_name = "Seeking Mode : \n\tStatus\t\t  : ";

    if (status == MUTUAL_GAZE_DETECTED)
    {
        ASSERT_TRUE(mutual_gaze_detected);
        test_name += "MUTUAL GAZE DETECTED";
    }
    else if (status == MUTUAL_GAZE_NOT_DETECTED)
    {
        ASSERT_FALSE(mutual_gaze_detected);
        test_name += "MUTUAL GAZE NOT DETECTED";
    }
    else {
        FAIL() << "Unrecognized status value: " + status;
        test_name += "UNRECOGNIZED STATUS";
    }
    
    mutual_gaze_detected = false;
    
    // Log the result of the test
    logTestResult(test_name, test_result);
}

/*  
 *  Function to run the location mode test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(OvertAttentionUnitTest, TestLocationMode) {
    if(!test_location_mode){
        GTEST_SKIP();
    }

    // Make service call to change mode and save the response from the service
    std::string output; 
    string service_call = "rosservice call /overtAttention/set_mode location 0 0 0";
    output = invoke_service(service_call.c_str());

    bool test_result = false;                   // Status of the test
    // Parse the integer from the response from the service
    int response;
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    LOCATION                    MODE                    ACTIVATED                   ";
            speech_pub.publish(message);
            // Publish the speech command
            message.data = "         X                                      0                                                                                                                          "  \
                                "          Y                                                            0                                                                                                    " \
                                "          Z                                                                    0                                                         ";
            speech_pub.publish(message);    
            ros::Duration(15).sleep();                  // put to sleep for a while
        }
        else{                         // Set the test result to false
            test_result = false;
            
            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    LOCATION                    MODE                    NOT                 ACTIVATED                   ";
            speech_pub.publish(message);
            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }

    ASSERT_TRUE(test_result);

    service_call = "rosservice call /overtAttention/set_mode location 3 3 1";
    output = invoke_service(service_call.c_str());

    // Parse the integer from the response from the service
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    LOCATION                    MODE                    ACTIVATED                   ";
            speech_pub.publish(message);

            // Publish the speech command
            message.data = "         X                                      3                                                                                                                              " \
                                "          Y                                                            3                                                                                                   "  \
                                "          Z                                                                    1                                                         ";
            speech_pub.publish(message);    
            ros::Duration(15).sleep();                  // put to sleep for a while
        }
        else{                         // Set the test result to false
            test_result = false;
            
            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    LOCATION                    MODE                    NOT                 ACTIVATED                   ";
            speech_pub.publish(message);
            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }
    
    // Log the result of the test
    std::string test_name = "Location Mode : ";
    logTestResult(test_name, test_result);
}

/*  
 *  Function to run the disabled mode test
 *  
 * @param:
 *     None
 * 
 * @return
 *    None
 */
TEST_F(OvertAttentionUnitTest, TestDisabledMode) {
    if(!test_disabled_mode){
        GTEST_SKIP();
    }

    // Make service call to change mode and save the response from the service
    std::string output; 
    string service_call = "rosservice call /overtAttention/set_mode disabled 0 0 0";
    output = invoke_service(service_call.c_str());

    bool test_result = false;                   // Status of the test
    // Parse the integer from the response from the service
    int response;
    if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
        if(response == 1){            // Check if the response is 1 and set the test result to true
            test_result = true;
            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    DISABLED                    MODE                    ACTIVATED                   ";;
            speech_pub.publish(message);

            ros::Duration(5).sleep();                  // put to sleep for a while
        }
        else{                         // Set the test result to false
            test_result = false;

            // publish message to speech topic
            std_msgs::String message;
            message.data = "                    DISABLED                    MODE                    NOT                 ACTIVATED                   ";;
            speech_pub.publish(message);

            ros::Duration(5).sleep();                  // put to sleep for a while
        }
    } else {
        FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
    }
    
    // Log the result of the test
    std::string test_name = "Disabled Mode : ";
    logTestResult(test_name, test_result);
    
}

/*
 *   Callback function for the face detection data received from the /face_detection/data topic
 *   The function receives the face detection data and determines mutual gaze
 */
void face_detection_data_received(const unit_tests::faceDetection& data_msg){
    
    // Store the status of the mutual gaze detection
    bool gaze_detected = false;

    // Loop through each face in the vector
    for (size_t i = 0; i < data_msg.mutualGaze.size(); ++i) {
        // check for mutual gaze with faces within 2 meters
        if (data_msg.mutualGaze[i] && data_msg.centroids[i].z <= 2.0) {
            gaze_detected = true;
            // cout << "mutual gaze:" << data_msg << "/n";
            break;
        } 
    }

    if(mutual_gaze_detected == false){                                  // Set the mutual gaze detected status based on the information from the face detection if it has been reset to false by the change mode service call
        mutual_gaze_detected = gaze_detected;                           // Set the mutual gaze detected status to true if any face has a mutual gaze
    }
}

/*
 *   Callback function for the overt attention mode data received from the /overtAttention/mode topic
 *   The function receives the overt attention mode data and determines the status
 */
void mode_data_received(const cssr_system::Status& mode_msg){
    status = mode_msg.value;
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
    bool topic_available = false;                                       // boolean to store if the topic is available
    ros::master::V_TopicInfo master_topics;                             // vector to store the topics
    ros::master::getTopics(master_topics);                              // get the topics

    // Iterate through the topics to check if the topic is available
    for (const auto& topic : master_topics){
        if (topic.name == topic_name){                                  // if the topic is found
            topic_available = true;                                     // set the topic as available
            break;
        }
    }

    return topic_available;                                             // return the topic availability
}
