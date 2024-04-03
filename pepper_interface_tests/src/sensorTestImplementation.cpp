/* sensorTestImplementation.cpp
*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa 
* Date: March 19, 2024
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


#include "pepper_interface_tests/sensorTest.h"

// Global variables to handle the output file 
bool output;
std::ofstream outputFile;
int timeDuration = 10;
std::string outputFilePath;

// Global variables to handle the audio file 
std::ofstream outAudio;
int totalSamples = 0;
std::string currentChannel = "rearLeft";

// Global variables to handle the video file 
bool saveVideo = true;
cv::VideoWriter videoWriter;
bool isVideoWriterInitialized = false;

void backSonar(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("BackSonar");
    output = true;

     // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for FrontCamera. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Subscribing to : " << topicName << "\n" ); 
    ros::Duration(1).sleep();

    // Subscribe to the back sonar topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, backSonarMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration); 
    ros::Time endTime = startTime + waitTime;  
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void frontSonar(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("FrontSonar");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for FrontSonar. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); 
    ros::Duration(1).sleep();

    // create a subscriber to the front sonar topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, frontSonarMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;  
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();  
    }
    rate.sleep();
}

void frontCamera(ros::NodeHandle nh) {
    // find the respective topic
    string topicName = extractTopic("FrontCamera");
    output = true;

    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for FrontCamera. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n");
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, frontCameraMessageReceived);

    ros::Rate rate(30);
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;
    
    while (ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    // Check if we have initialized the video writer and release it
    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Front Camera");
}

void bottomCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("BottomCamera");
    output = true;

    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for BottomCamera. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n");
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, bottomCameraMessageReceived);

    ros::Rate rate(30);
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;

    while (ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    // Check if we have initialized the video writer and release it
    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Bottom Camera");
}

void depthCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("DepthCamera");
    output = true;

    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for DepthCamera. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n");
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, depthCameraMessageReceived);

    ros::Rate rate(30);
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;

    while (ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    // Check if we have initialized the video writer and release it
    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Depth Camera");
}

void laserSensor(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("Laser");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Laser. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    ros::Subscriber sub = nh.subscribe(topicName, 1, laserSensorMessageReceived);
    
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void odom(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("Odometry");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Odometry. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    ros::Subscriber sub = nh.subscribe(topicName, 1, odomMessageReceived);
    
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now();             // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void imu(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("IMU");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for IMU. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    ros::Subscriber sub = nh.subscribe(topicName, 1, imuMessageReceived);
    
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now();                         
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void jointState(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("JointState");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for JointState. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    ros::Subscriber sub = nh.subscribe(topicName, 1, jointStateMessageReceived);
    
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

// compile only if PEPPER_ROBOT is defined
#ifdef PEPPER_ROBOT
void speech(ros::NodeHandle nh){
    // Assuming extractTopic is a custom function that returns a std::string
    std::string topicName = extractTopic("Speech");

    // Publish the speech message 
    ros::Publisher pub = nh.advertise<std_msgs::String>(topicName, 1);
    ros::Duration(1).sleep();                   // Wait for the connection to establish

    std_msgs::String msg;
    msg.data = "This is the CSSR4Africa speaker test.";
    pub.publish(msg);
    ros::spinOnce();                            // Process incoming messages once. Not typically necessary for a publisher only.
    ros::Duration(1).sleep(); 
}


void stereoCamera(ros::NodeHandle nh){
    // find the respective topic
    string topicName = extractTopic("StereoCamera");
    output = true;

    // check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for StereoCamera. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, stereoCameraMessageReceived);

    // Listen for incoming messages and execute the callback function
    ros::Rate rate(30);
    ros::Time startTime = ros::Time::now(); // start now
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;

    while (ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    // Check if we have initialized the video writer and release it
    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Stereo Camera");
}

void microphone(ros::NodeHandle nh) {
    std::string topicName = extractTopic("Microphone");
    int sampleRate = 48000;

    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Microphone. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test\n");
    ros::Duration(1).sleep();

    #ifdef ROS
        outAudio.open(ros::package::getPath(ROS_PACKAGE_NAME) + "/data/microphoneOutput.wav", std::ios::binary);
    #else
        ROS_INFO_STREAM("Unable to open the output file microphoneOutput.wav\n");
        promptAndExit(1);
    #endif

    ros::Subscriber sub = nh.subscribe(topicName, 1000, microphoneMessageReceived);
    
    for (int i = 0; i < 4; ++i) { // Four channels to record

        ROS_INFO_STREAM("Recording channel " << currentChannel << "\n");
        
        ros::Time startTime = ros::Time::now();
        ros::Time endTime = startTime + ros::Duration(timeDuration);

        while (ros::ok() && ros::Time::now() < endTime) {
            ros::spinOnce();
        }

        switchMicrophoneChannel(); // Switch to the next channel

    }

    outAudio.seekp(0, std::ios::beg);
    writeWavHeader(outAudio, sampleRate, totalSamples);

    outAudio.close();
    ROS_INFO_STREAM("Microphone test finished\n");

    playAndDeleteFile();
}

// callback function to process the received microphone message
void microphoneMessageReceived(const naoqi_driver::AudioCustomMsg& msg) {
    if (currentChannel.empty() || !outAudio.is_open()) {
        return;
    }

    const std::vector<int16_t>* channelData = nullptr;

    if (currentChannel == "frontLeft") {
        channelData = &msg.frontLeft;
    } else if (currentChannel == "frontRight") {
        channelData = &msg.frontRight;
    } else if (currentChannel == "rearLeft") {
        channelData = &msg.rearLeft;
    } else if (currentChannel == "rearRight") {
        channelData = &msg.rearRight;
    }

    if (channelData) {
        for (const auto& sample : *channelData) {
            outAudio.write(reinterpret_cast<const char*>(&sample), sizeof(sample));
            totalSamples++;
        }
    }
}

// Callback function to process the received stereo camera image message
void stereoCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract the path of the ROS package
    string path;
    string videoPath;

    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    if (output == true) {
        // Set the path for the text output file
        outputFilePath = path + "/data/sensorTestOutput.dat";

        // Set the path for the video file the first time the output is true
        if (!isVideoWriterInitialized) {
            videoPath = path + "/data/stereoCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        outputFile << "[TESTING] ---- STEREO CAMERA ----\n\n";
        outputFile << "[MESSAGES] Printing stereo camera data information received.\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        outputFile.close();
        output = false;
    }

    if (!isVideoWriterInitialized) {
        // Initialize video writer with the path set for video output
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);
        
        if (!videoWriter.isOpened()) {
            ROS_ERROR_STREAM("Failed to initialize video writer with path: " << videoPath);
            promptAndExit(1);
        }
        
        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Stereo Camera", img);
    cv::waitKey(30);
}    
#endif

// Callback function to process the received joint state message
void jointStateMessageReceived(const sensor_msgs::JointState& msg) {
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing joint state data received.\n");
    // Print the received message attributes
    ROS_INFO_STREAM("Header: " << msg.header << "\n" );
    for (size_t i = 0; i < msg.name.size(); ++i) {
        std::cout << "Name: " << msg.name[i] << "\n"
                  << "Position: ";
        if (std::isnan(msg.position[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.position[i];
        }
        std::cout << "\nVelocity: ";
        if (std::isnan(msg.velocity[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.velocity[i];
        }
        std::cout << "\nEffort: ";
        if (std::isnan(msg.effort[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.effort[i];
        }
        std::cout << "\n\n";
    }
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

        
    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the message received in an output file if the output variable is true
    if (output == true){
        // complete the path of the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // write on the output file
        outputFile << "[TESTING] ---- JOINT STATE ----\n\n";
        outputFile << "[MESSAGES] Printing joint state data received.\n";
        outputFile << "Header: " << msg.header << "\n" ;
        for (size_t i = 0; i < msg.name.size(); ++i) {
        std::cout << "Name: " << msg.name[i] << "\n"
                  << "Position: ";
        if (std::isnan(msg.position[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.position[i];
        }
        std::cout << "\nVelocity: ";
        if (std::isnan(msg.velocity[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.velocity[i];
        }
        std::cout << "\nEffort: ";
        if (std::isnan(msg.effort[i])) {
            std::cout << "NaN";
        } else {
            std::cout << std::setprecision(9) << msg.effort[i];
        }
        std::cout << "\n\n";
    }
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

void odomMessageReceived(const nav_msgs::Odometry& msg){
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing odometry data received.\n");
    // Print the received message attributes
    ROS_INFO_STREAM("Header: " << msg.header << "\n" );
    ROS_INFO_STREAM("Child frame id: " << msg.child_frame_id << "\n" );
    ROS_INFO_STREAM("Pose: " << msg.pose << "\n" );
    ROS_INFO_STREAM("Twist: " << msg.twist << "\n" );
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");
    
    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif
        

    // Write the message received in an output file if the output variable is true
    if (output == true){
        
        // complete the path of the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // write on the output file
        outputFile << "[TESTING] ---- ODOMETRY ----\n\n";
        outputFile << "[MESSAGES] Printing odometry data received.\n";
        outputFile << "Header: " << msg.header << "\n" ;
        outputFile << "Child frame id: " << msg.child_frame_id << "\n" ;
        outputFile << "Pose: " << msg.pose << "\n" ;
        outputFile << "Twist: " << msg.twist << "\n" ;
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

void imuMessageReceived(const sensor_msgs::Imu& msg) {
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing IMU data received.\n");
    // Print the received message attributes
    ROS_INFO_STREAM("Header: " << msg.header << "\n" );
    ROS_INFO_STREAM("Orientation: " << msg.orientation << "\n" );
    ROS_INFO_STREAM("Angular velocity: " << msg.angular_velocity << "\n" );
    ROS_INFO_STREAM("Linear acceleration: " << msg.linear_acceleration << "\n" );
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif
    
    // Write the message received in an output file if the output variable is true
    if (output == true){
        
        // complete the path of the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // write on the output file
        outputFile << "[TESTING] ---- IMU ----\n\n";
        outputFile << "[MESSAGES] Printing IMU data received.\n";
        outputFile << "Header: " << msg.header << "\n" ;
        outputFile << "Orientation: " << msg.orientation << "\n" ;
        outputFile << "Angular velocity: " << msg.angular_velocity << "\n" ;
        outputFile << "Linear acceleration: " << msg.linear_acceleration << "\n" ;
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}


// Callback function to process the received sonar message
void backSonarMessageReceived(const sensor_msgs::Range& msg) {
    string path;

    // Print a message indicating that sonar data is being printed
    ROS_INFO_STREAM("[MESSAGES] Printing back sonar data received.\n");

    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );          // Print the frame ID of the received message
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" );       // Print the field of view of the sonar sensor
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" );     // Print the minimum range value reported by the sonar sensor
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" );     // Print the maximum range value reported by the sonar sensor
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" );                 // Print the received range value reported by the sonar sensor
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");                 // Print a message indicating the end of printing sonar data
    
    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the message received in an output file if the output variable is true
    if (output == true){
        // complete the path of the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }
        
        // write on the output file
        outputFile << "[TESTING] ---- BACK SONAR ----\n\n";
        outputFile << "[MESSAGES] Printing back sonar data received.\n";
        outputFile << "Frame id: " << msg.header.frame_id << "\n";
        outputFile << "Field of view: " << msg.field_of_view << "\n";
        outputFile << "Minimum range value: " << msg.min_range << "\n";
        outputFile << "Maximum range value: " << msg.max_range << "\n";
        outputFile << "Range value: " << msg.range << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

// Callback function to process the received front sonar message
void frontSonarMessageReceived(const sensor_msgs::Range& msg) {
    string path;

    // Print a message indicating that sonar data is being printed
    ROS_INFO_STREAM("[MESSAGES] Printing front sonar data received.\n");

    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );          
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" );       
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" );     
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" );     
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" );                 
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");                 

    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the message received in an output file if the output variable is true
    if (output == true){
        // complete the path of the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }
        
        // write on the output file
        outputFile << "[TESTING] ---- FRONT SONAR ----\n\n";
        outputFile << "[MESSAGES] Printing front sonar data received.\n";
        outputFile << "Frame id: " << msg.header.frame_id << "\n";
        outputFile << "Field of view: " << msg.field_of_view << "\n";
        outputFile << "Minimum range value: " << msg.min_range << "\n";
        outputFile << "Maximum range value: " << msg.max_range << "\n";
        outputFile << "Range value: " << msg.range << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract the path of the ROS package
    string path;
    string videoPath;
        
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif
   
    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    if (output == true) {
        // Set the path for the text output file
        outputFilePath = path + "/data/sensorTestOutput.dat";

        // Set the path for the video file the first time the output is true
        if (!isVideoWriterInitialized) {
            videoPath = path + "/data/frontCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        outputFile << "[TESTING] ---- FRONT CAMERA ----\n\n";
        outputFile << "[MESSAGES] Printing front camera data information received.\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        outputFile.close();
        output = false;
    }

    if (!isVideoWriterInitialized) {
        // Initialize video writer with the path set for video output
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);
        
        if (!videoWriter.isOpened()) {
            ROS_ERROR_STREAM("Failed to initialize video writer with path: " << videoPath);
            promptAndExit(1);
        }
        
        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Front Camera", img);
    cv::waitKey(30);
}

// Callback function to process the received bottom camera image message
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    // Extract the path of the ROS package
    string path;
    string videoPath;

    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1); 
    #endif

    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    if (output == true) {
        // Set the path for the text output file
        outputFilePath = path + "/data/sensorTestOutput.dat";

        // Set the path for the video file the first time the output is true
        if (!isVideoWriterInitialized) {
            videoPath = path + "/data/bottomCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        outputFile << "[TESTING] ---- BOTTOM CAMERA ----\n\n";
        outputFile << "[MESSAGES] Printing bottom camera data information received.\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        outputFile.close();
        output = false;
    }

    if (!isVideoWriterInitialized) {
        // Initialize video writer with the path set for video output
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);
        
        if (!videoWriter.isOpened()) {
            ROS_ERROR_STREAM("Failed to initialize video writer with path: " << videoPath);
            promptAndExit(1);
        }
        
        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Bottom Camera", img);
    cv::waitKey(30);
}

// Callback function to process the received depth camera image message
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    //Extract the path of the ROS package
    string path;
    string videoPath;

    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    ROS_INFO("[MESSAGE] Image received has a width: %d and height: %d", imgWidth, imgHeight);

    if (output == true) {
        // Set the path for the text output file
        outputFilePath = path + "/data/sensorTestOutput.dat";

        // Set the path for the video file the first time the output is true
        if (!isVideoWriterInitialized) {
            videoPath = path + "/data/depthCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        outputFile << "[TESTING] ---- DEPTH CAMERA ----\n\n";
        outputFile << "[MESSAGES] Printing depth camera data information received.\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        outputFile.close();
        output = false;
    }

    if (!isVideoWriterInitialized) {
        // Initialize video writer with the path set for video output
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);
        
        if (!videoWriter.isOpened()) {
            ROS_ERROR_STREAM("Failed to initialize video writer with path: " << videoPath);
            promptAndExit(1);
        }
        
        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat img = cv_ptr->image;

    double min = 0;
    double max = 1000;

    cv::Mat img_scaled_8u;
    cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
    cv::Mat color_img;

    if(img_scaled_8u.type() ==  CV_8UC1){
        cv::cvtColor(img_scaled_8u, color_img, CV_GRAY2RGB); 
    }

    // Write frame to video
    videoWriter.write(color_img);

    cv::imshow("Depth Camera", color_img);
    cv::waitKey(30);
}

// Callback function to process the received laser sensor message
void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg) {
    string path;
    
    ROS_INFO_STREAM("[MESSAGES] Printing laser sensor data received.\n");
    // Print the received message attributes
    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );
    ROS_INFO_STREAM("Start angle of the scan: " << msg.angle_min << "\n" );
    ROS_INFO_STREAM("End angle of the scan: " << msg.angle_max << "\n" );
    ROS_INFO_STREAM("Angular distance between measurements: " << msg.angle_increment << "\n" );
    ROS_INFO_STREAM("Time between measurements: " << msg.time_increment << "\n" );
    ROS_INFO_STREAM("Time between scans: " << msg.scan_time << "\n" );
    ROS_INFO_STREAM("Minimum range value: " << msg.range_min << "\n" );
    ROS_INFO_STREAM("Maximum range value: " << msg.range_max << "\n" );
    ROS_INFO_STREAM("Range data: (size: " << msg.ranges.size() << ") \n" );
            
    for (auto rng : msg.ranges){
        ROS_INFO_STREAM(rng);
    }
    ROS_INFO_STREAM("\n");
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

    
    // set the main path for the output file
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the message received in an output file if the output variable is true
    if (output == true){
        // complete the path of the output file
        path += "/data/sensorTestOutput.dat";
        
        // open the output file
        outputFile.open(path.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // write on the output file
        outputFile << "[TESTING] ---- LASER SENSOR ----\n\n";
        outputFile << "[MESSAGES] Printing laser sensor data received.\n";
        outputFile << "Frame id: " << msg.header.frame_id << "\n" ;
        outputFile << "Start angle of the scan: " << msg.angle_min << "\n" ;
        outputFile << "End angle of the scan: " << msg.angle_max << "\n" ;
        outputFile << "Angular distance between measurements: " << msg.angle_increment << "\n" ;
        outputFile << "Time between measurements: " << msg.time_increment << "\n" ;
        outputFile << "Time between scans: " << msg.scan_time << "\n" ;
        outputFile << "Minimum range value: " << msg.range_min << "\n" ;
        outputFile << "Maximum range value: " << msg.range_max << "\n" ;
        outputFile << "Range data: (size: " << msg.ranges.size() << ") \n" ;
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // close the output file
        outputFile.close();

        // set the output to false so that only the first received message will be written to the output file
        output = false;
    }
}

/* Helper Functions */
void promptAndExit(int status){
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void promptAndContinue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

/* Extract topic names for the respective simulator or physical robot */
string extractTopic(string key){
    bool debug = false;   // used to turn debug message on
    
    std::string configFileName      = "sensorTestConfiguration.ini";        // configuration filename
    std::string packagePath;                                                // ROS package path
    std::string configPathFile;                                             // configuration path and filename
    
    std::string platformKey         = "platform";                           // platform key 
    std::string robotTopicKey       = "robotTopics";                        // robot topic key
    std::string simulatorTopicKey   = "simulatorTopics";                    // simulator topic key

    std::string platformValue;                                              // platform value
    std::string robotTopicValue;                                            // robot topic value
    std::string simulatorTopicValue;                                        // simulator topic value
    
    std::string topicFileName;                                              // topic filename
    std::string topicPathFile;                                              // topic with path and file 

    std::string topic_value = "";                                           // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
        promptAndExit(1);
    #endif

    // set configuration path
    configPathFile = packagePath + "/config/" + configFileName;


    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        
        if (paramKey == platformKey){ platformValue = paramValue;}
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}
        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}
    }
    
    configFile.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topicFileName.c_str());

    // set the topic path and file
    topicPathFile = packagePath + "/config/" + topicFileName;

    if (debug) printf("Topic file is %s\n", topicPathFile.c_str());

    // Open topic file
    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()){
        printf("Unable to open the topic file %s\n", topicPathFile.c_str());
        promptAndExit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topicFile, topicLineRead)){
        std::istringstream iss(topicLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }
    topicFile.close();

    return topic_value;
}

/* Extract the expected tests to run for the respective actuator or sensor tests */
std::vector<std::string> extractTests(std::string set){
    bool debug = false;   // used to turn debug message on
    
    std::string inputFileName = "sensorTestInput.ini";  // input filename
    std::string inputPath;                              // input path
    std::string inputPathFile;                          // input path and filename
    
    std::vector<std::string> testName;
    std::string flag;

    // Construct the full path of the input file
    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        // print out error message and exit if the package name is not defined
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
        promptAndExit(1);
    #endif

    // set input path
    inputPathFile = inputPath + "/config/" + inputFileName;

    if (debug) printf("Input file is %s\n", inputPathFile.c_str());

    // Open input file
    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()){
        printf("Unable to open the input file %s\n", inputPathFile.c_str());
        promptAndExit(1);
    }

    std::string inpLineRead;            // variable to read the line in the file
    
    std::string paramKey, paramValue;   // variables to keep the key value pairs read

    // Get key-value pairs from the input file
    while(std::getline(inputFile, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue);                                                                   // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);     // convert to lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);           // convert to lower case

        if (paramValue == "true"){ testName.push_back(paramKey);}
    }
    inputFile.close();

    return testName;
}

// Extract the mode to run the tests
std::string extractMode(){
    bool debug = false;   // used to turn debug message on

    std::string configFileName = "sensorTestConfiguration.ini";     // configuration filename
    std::string packagePath;                                        // ROS package path
    std::string configPathFile;                                     // configuration path and filename

    std::string modeKey        = "mode";                            // mode key

    std::string modeValue;                                          // mode value

    // Construct the full path of the configuration file
    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
    #endif

    // set configuration path
    configPathFile = packagePath + "/config/" + configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    
    // Get key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // To lower case
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == modeKey){ modeValue = paramValue;}
    }
    configFile.close();

    // verify the modeValue is not empty
    if (modeValue == ""){
        printf("Unable to find a valid mode.\n");
        promptAndExit(1);
    }
    return modeValue;
}

// This function updates the global variable for the current microphone channel to record.
void switchMicrophoneChannel() {
    if (currentChannel == "rearLeft") {
        currentChannel = "rearRight";
    } else if (currentChannel == "rearRight") {
        currentChannel = "frontLeft";
    } else if (currentChannel == "frontLeft") {
        currentChannel = "frontRight";
    } else {
        currentChannel = ""; // Finished recording all channels
    }
}

// Write a WAV header to the output file
void writeWavHeader(std::ofstream& file, int sampleRate, int numSamples) {
    int byteRate = sampleRate * 2; // 16-bit mono = 2 bytes per sample
    int dataSize = numSamples * 2; // Total number of bytes in data
    int chunkSize = 36 + dataSize;
    
    file.write("RIFF", 4); // ChunkID
    file.write(reinterpret_cast<const char*>(&chunkSize), 4); // ChunkSize
    file.write("WAVE", 4); // Format
    file.write("fmt ", 4); // Subchunk1ID
    int subChunk1Size = 16; // PCM header size
    file.write(reinterpret_cast<const char*>(&subChunk1Size), 4); // Subchunk1Size
    short audioFormat = 1; // PCM = 1
    file.write(reinterpret_cast<const char*>(&audioFormat), 2); // AudioFormat
    short numChannels = 1; // Mono = 1, Stereo = 2
    file.write(reinterpret_cast<const char*>(&numChannels), 2); // NumChannels
    file.write(reinterpret_cast<const char*>(&sampleRate), 4); // SampleRate
    file.write(reinterpret_cast<const char*>(&byteRate), 4); // ByteRate
    short blockAlign = 2; // NumChannels * BitsPerSample/8
    file.write(reinterpret_cast<const char*>(&blockAlign), 2); // BlockAlign
    short bitsPerSample = 16; // Bits per sample
    file.write(reinterpret_cast<const char*>(&bitsPerSample), 2); // BitsPerSample
    file.write("data", 4); // Subchunk2ID
    file.write(reinterpret_cast<const char*>(&dataSize), 4); // Subchunk2Size
}

void playAndDeleteFile() {
    // use True/False to delete the file after playing
    bool deleteFile = false;
    
    // check if the file exists
    std::string fileName = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/microphoneOutput.wav";

    // check if the file exists
    std::ifstream file(fileName);
    if (!file.good()) {
        std::cerr << "Error: File not found: " << fileName << std::endl;
        return;
    }

    // Play the audio file
    if (std::system(("aplay " + fileName).c_str()) != 0) {
        std::cerr << "Error playing file: " << fileName << std::endl;
        return; // Exit if playing failed
    }

    if (deleteFile) {
        if (std::remove(fileName.c_str()) != 0) {
            std::cerr << "Error deleting file: " << fileName << std::endl;
        } else {
            std::cout << "File deleted successfully: " << fileName << std::endl;
        }
    }
}

void initializeOutputFile(std::ofstream& outputFile, const std::string& path) { 
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        std::cerr << "Unable to open the output file " << path << "\n";
        throw std::runtime_error("Failed to open output file.");
    }

    outputFile << "[TESTING] ############ SENSORS ############\n\n";
    
    // Assuming getCurrentTime() is implemented elsewhere
    outputFile << "[START TIME] " << getCurrentTime() << "\n";
    
    outputFile.close();
}

void finalizeOutputFile(std::ofstream& outputFile, const std::string& path) {
    outputFile.open(path, std::ofstream::app);
    outputFile << "[END TIME] " << getCurrentTime() << "\n\n";
    outputFile.close();
}

std::string getCurrentTime() {
    char buffer[50];
    std::time_t now = std::time(0);
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d.%X", std::localtime(&now));
    return std::string(buffer);
}

std::string getOutputFilePath() {
    std::string basePath;
    std::string fileName = "sensorTestOutput.dat";

    #ifdef ROS
    // If ROS is defined, use the ROS package path
    basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    
    // Append the subdirectory and filename to the base path
    basePath += "/data/";
    #else
    
    // If not running within ROS, output an error message and exit
    std::cerr << "Error: ROS environment not detected. Unable to determine the output file path." << std::endl;
    exit(EXIT_FAILURE);
    #endif

    return basePath + fileName;
}

void executeTestsSequentially(const std::vector<std::string>& testNames, ros::NodeHandle& nh) {
    // Map test names to their corresponding functions
    std::unordered_map<std::string, TestFunction> testMap = {
        {"backsonar", backSonar},
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera},
        {"bottomcamera", bottomCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu}
    };

    #ifdef PEPPER_ROBOT
    testMap["stereocamera"] = stereoCamera;
    testMap["microphone"] = microphone;
    testMap["speech"] = speech;
    #endif


    for (const auto& testName : testNames) {
        auto it = testMap.find(testName);
        if (it != testMap.end()) {
            // Call the function associated with testName
            it->second(nh);
        } else {
            std::cerr << "Unknown test provided: " << testName << ". Proceeding to the next test...\n";
        }
    }
}

void executeTestsInParallel(const std::vector<std::string>& testNames, ros::NodeHandle& nh) {
    std::unordered_map<std::string, TestFunction> testMap = {
        {"backsonar", backSonar},
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera},   
        {"bottomcamera", bottomCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu},
    };

    #ifdef PEPPER_ROBOT
    testMap["stereocamera"] = stereoCamera;
    testMap["microphone"] = microphone;
    testMap["speech"] = speech;
    #endif

    std::vector<std::thread> threads;

    // Create a thread for each test found in the map
    for (const auto& testName : testNames) {
        auto it = testMap.find(testName);
        if (it != testMap.end()) {
            threads.emplace_back(it->second, std::ref(nh));
        } else {
            std::cerr << "There is no test function associated with the test name: " << testName << "Proceeding to the next test...\n";
        }
    }

    // Wait for all threads to finish execution
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}