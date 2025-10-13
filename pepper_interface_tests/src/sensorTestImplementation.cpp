/* sensorTestImplementation.cpp Implementation code to test the sensors of the Pepper robot using ROS interface.
*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa
* Date: October 13, 2025
* Version: v1.1
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

#include "pepper_interface_tests/sensorTestInterface.h"

// Global variables to handle the output file
bool verboseMode = false;
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

struct WriterEntry {
    cv::VideoWriter writer;
    bool opened = false;
};

static std::mutex g_writer_mtx;
static std::unordered_map<std::string, WriterEntry> g_writers;

static WriterEntry& getOrOpenWriter(const std::string& key,
                                    const std::string& videoPath,
                                    int width, int height,
                                    double fps,
                                    bool isColor = true)
{
    std::lock_guard<std::mutex> lk(g_writer_mtx);
    auto& entry = g_writers[key];
    if (!entry.opened) {
        entry.writer.open(videoPath,
                          cv::VideoWriter::fourcc('m','p','4','v'),
                          fps,
                          cv::Size(width, height),
                          isColor);
        if (!entry.writer.isOpened()) {
            ROS_ERROR_STREAM("Failed to initialize video writer with path: " << videoPath);
        } else {
            entry.opened = true;
        }
    }
    return entry;
}

static void releaseWriter(const std::string& key) {
    std::lock_guard<std::mutex> lk(g_writer_mtx);
    auto it = g_writers.find(key);
    if (it != g_writers.end()) {
        if (it->second.opened) it->second.writer.release();
        g_writers.erase(it);
    }
}

// ---------- Single UI thread that owns all cv::imshow ----------
class FrameBus {
public:
  void start(bool show=true, int wait_ms=1) {
    if (!show) return;
    running_.store(true);
    ui_ = std::thread([this, wait_ms]{
      std::unordered_set<std::string> created;
      std::unique_lock<std::mutex> lk(m_);
      while (running_.load()) {
        cv_.wait_for(lk, std::chrono::milliseconds(50),
                     [this]{ return dirty_ || !running_.load(); });
        if (!running_.load()) break;

        auto snapshot = latest_;   // shallow copy of map and Mats
        dirty_ = false;
        lk.unlock();

        for (auto &kv : snapshot) {
          const std::string &name = kv.first;
          const cv::Mat &img = kv.second;
          if (img.empty()) continue;
          if (!created.count(name)) {
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, std::max(320, img.cols), std::max(240, img.rows));
            created.insert(name);
          }
          cv::imshow(name, img);
        }
        cv::waitKey(wait_ms);
        lk.lock();
      }
      for (auto &kv : latest_) cv::destroyWindow(kv.first);
    });
  }

  void stop() {
    running_.store(false);
    cv_.notify_all();
    if (ui_.joinable()) ui_.join();
  }

  void publish(const std::string& name, const cv::Mat& bgr) {
    std::lock_guard<std::mutex> lk(m_);
    latest_[name] = bgr.clone();
    dirty_ = true;
    cv_.notify_one();
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  std::unordered_map<std::string, cv::Mat> latest_;
  std::thread ui_;
  std::atomic<bool> running_{false};
  bool dirty_ = false;
};

static FrameBus g_frameBus;

static inline std::string rstrip_slash(std::string s) {
    while (!s.empty() && s.back() == '/') s.pop_back();
    return s;
}

// Strip leading '/' from a ROS node name
std::string cleanNodeName(const std::string& name) {
    return (!name.empty() && name.front() == '/') ? name.substr(1) : name;
}

// 10-second heartbeat
void heartbeatCb(const ros::TimerEvent&) {
    ROS_INFO_STREAM( cleanNodeName(ros::this_node::getName()) << ": running..." );
}

static inline std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}



void backSonar(ros::NodeHandle nh){
    /*
     * Function to test the back sonar sensor by subscribing to its topic and receiving messages
     * The function subscribes to the back sonar topic and listens for incoming messages for a specified duration
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the back sonar sensor
    string topicName = extractTopic("BackSonar");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for BackSonar. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Subscribing to : " << topicName << "\n" ); 
    ros::Duration(1).sleep();

    // Subscribe to the back sonar topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, backSonarMessageReceived);

    // Listen for incoming messages and execute the callback function for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time
    ros::Duration waitTime = ros::Duration(timeDuration); 
    ros::Time endTime = startTime + waitTime;  // Calculate the end time
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void frontSonar(ros::NodeHandle nh){
    /*
     * Function to test the front sonar sensor by subscribing to its topic and receiving messages
     * The function subscribes to the front sonar topic and listens for incoming messages for a specified duration
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the front sonar sensor
    string topicName = extractTopic("FrontSonar");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for FrontSonar. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n"  ); 
    ros::Duration(1).sleep();

    // Create a subscriber to the front sonar topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, frontSonarMessageReceived);

    // Listen for incoming messages and execute the callback function for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time
    ros::Duration waitTime = ros::Duration(timeDuration);
    ros::Time endTime = startTime + waitTime;  // Calculate the end time
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();  
    }
    rate.sleep();
}

void frontCamera(ros::NodeHandle nh) {
    /*
     * Function to test the front camera sensor by subscribing to its image topic and receiving messages
     * The function subscribes to the front camera topic, displays images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the front camera sensor
     std::string topicName = extractTopic("FrontCamera");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) { ROS_WARN_STREAM("No valid topic for FrontCamera"); return; }

    ROS_INFO_STREAM("Subscribing to :" << topicName);
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, frontCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time endTime = ros::Time::now() + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) { ros::spinOnce(); rate.sleep(); }

    releaseWriter("front_camera");
    cv::destroyWindow("Front Camera");
}

void bottomCamera(ros::NodeHandle nh){
    /*
     * Function to test the bottom camera sensor by subscribing to its image topic and receiving messages
     * The function subscribes to the bottom camera topic, displays images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the bottom camera sensor
    std::string topicName = extractTopic("BottomCamera");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) { ROS_WARN_STREAM("No valid topic for BottomCamera"); return; }

    ROS_INFO_STREAM("Subscribing to :" << topicName);
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, bottomCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time endTime = ros::Time::now() + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) { ros::spinOnce(); rate.sleep(); }

    releaseWriter("bottom_camera");
    cv::destroyWindow("Bottom Camera");
}

void realsenseRGBCamera(ros::NodeHandle nh) {
    /*
     * Function to test the Intel RealSense RGB camera by subscribing to its image topic and receiving messages
     * The function subscribes to the RealSense RGB camera topic, displays images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the RealSense RGB camera sensor
    std::string topicName = extractTopic("RealSenseCameraRGB");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for RealSenseRGB. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Subscribing to :" << topicName << "\n");
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, realsenseRGBCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time startTime = ros::Time::now();
    const ros::Time endTime   = startTime + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }

    // NEW: release this stream’s writer and close the window
    releaseWriter("realsense_rgb_camera");
    cv::destroyWindow("RealSense RGB Camera");
}

void depthCamera(ros::NodeHandle nh){
    /*
     * Function to test the depth camera sensor by subscribing to its image topic and receiving messages
     * The function subscribes to the depth camera topic, displays depth images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the depth camera sensor
    std::string topicName = extractTopic("DepthCamera");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) { ROS_WARN_STREAM("No valid topic for DepthCamera"); return; }

    ROS_INFO_STREAM("Subscribing to :" << topicName);
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, depthCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time endTime = ros::Time::now() + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) { ros::spinOnce(); rate.sleep(); }

    releaseWriter("depth_camera");
    cv::destroyWindow("Depth Camera");
}

void realsenseDepthCamera(ros::NodeHandle nh) {
    /*
     * Function to test the Intel RealSense depth camera by subscribing to its depth image topic and receiving messages
     * The function subscribes to the RealSense depth camera topic, displays depth images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the RealSense depth camera sensor
    std::string topicName = extractTopic("RealSenseCameraDepth");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) { ROS_WARN_STREAM("No valid topic for RealSenseDepth"); return; }

    ROS_INFO_STREAM("Subscribing to :" << topicName);
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, realsenseDepthCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time endTime = ros::Time::now() + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) { ros::spinOnce(); rate.sleep(); }

    releaseWriter("realsense_depth_camera");
    cv::destroyWindow("RealSense Depth Camera");
}

void laserSensor(ros::NodeHandle nh){
    /*
     * Function to test the laser sensor by subscribing to its topic and receiving messages
     * The function subscribes to the laser sensor topic and listens for incoming laser scan messages
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the laser sensor
    string topicName = extractTopic("Laser");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Laser. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    // Subscribe to the laser sensor topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, laserSensorMessageReceived);
    
    // Listen for incoming laser scan messages for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   // Calculate the end time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void odom(ros::NodeHandle nh){
    /*
     * Function to test the odometry sensor by subscribing to its topic and receiving messages
     * The function subscribes to the odometry topic and listens for incoming position and velocity data
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the odometry sensor
    string topicName = extractTopic("Odometry");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Odometry. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    // Subscribe to the odometry topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, odomMessageReceived);
    
    // Listen for incoming odometry messages for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time          
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   // Calculate the end time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void imu(ros::NodeHandle nh){
    /*
     * Function to test the IMU sensor by subscribing to its topic and receiving messages
     * The function subscribes to the IMU topic and listens for incoming acceleration, angular velocity, and orientation data
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the IMU sensor
    string topicName = extractTopic("IMU");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for IMU. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    // Subscribe to the IMU topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, imuMessageReceived);
    
    // Listen for incoming IMU messages for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time                      
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   // Calculate the end time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

void jointState(ros::NodeHandle nh){
    /*
     * Function to test the joint state sensor by subscribing to its topic and receiving messages
     * The function subscribes to the joint state topic and listens for incoming joint position, velocity, and effort data
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the joint state sensor
    string topicName = extractTopic("JointState");
    checkTopicAvailable(topicName);
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for JointState. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test \n"  ); 
    ros::Duration(1).sleep();
    
    // Subscribe to the joint state topic and associate it with the callback function
    ros::Subscriber sub = nh.subscribe(topicName, 1, jointStateMessageReceived);
    
    // Listen for incoming joint state messages for the specified time duration
    ros::Rate rate(30); 
    ros::Time startTime = ros::Time::now(); // Record the start time
    ros::Duration waitTime = ros::Duration(timeDuration);  
    ros::Time endTime = startTime + waitTime;   // Calculate the end time
    
    while(ros::ok() && ros::Time::now() < endTime) {
        ros::spinOnce();
        rate.sleep();
    }
}

// compile only if PEPPER_ROBOT is defined
#ifdef PEPPER_ROBOT
void speech(ros::NodeHandle nh){
    /*
     * Function to test the speech output by publishing a test message to the speech topic
     * The function publishes a predefined test message to verify the speech functionality
     *
     * @param:
     *     nh: ROS node handle used for publishing to topics
     *
     * @return:
     *     None
     */
    
    // Extract the topic name for speech output using the custom extractTopic function
    std::string topicName = extractTopic("Speech");
    checkTopicAvailable(topicName);

    // Create a publisher for the speech message topic
    ros::Publisher pub = nh.advertise<std_msgs::String>(topicName, 1);
    ros::Duration(1).sleep();                   // Wait for the connection to establish

    // Create and publish the test speech message
    std_msgs::String msg;
    msg.data = "This is the CSSR4Africa speaker test.";
    pub.publish(msg);
    ros::spinOnce();                            // Process incoming messages once. Not typically necessary for a publisher only.
    ros::Duration(1).sleep(); 
}


void stereoCamera(ros::NodeHandle nh){
    /*
     * Function to test the stereo camera sensor by subscribing to its image topic and receiving messages
     * The function subscribes to the stereo camera topic, displays stereo images, and optionally saves video output
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics and image transport
     *
     * @return:
     *     None
     */
    
    // Find the respective topic for the stereo camera sensor
    std::string topicName = extractTopic("StereoCamera");
    checkTopicAvailable(topicName);
    output = true;

    if (topicName.empty()) { ROS_WARN_STREAM("No valid topic for StereoCamera"); return; }

    ROS_INFO_STREAM("Subscribing to :" << topicName);
    ros::Duration(1).sleep();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName, 1, stereoCameraMessageReceived);

    ros::Rate rate(30);
    const ros::Time endTime = ros::Time::now() + ros::Duration(timeDuration);
    while (ros::ok() && ros::Time::now() < endTime) { ros::spinOnce(); rate.sleep(); }

    releaseWriter("stereo_camera");
    cv::destroyWindow("Stereo Camera");
}

void microphone(ros::NodeHandle nh) {
    /*
     * Function to test the microphone sensor by recording audio from multiple channels
     * The function records audio from four different microphone channels and saves the output as a WAV file
     *
     * @param:
     *     nh: ROS node handle used for subscribing to topics
     *
     * @return:
     *     None
     */
    
    // Extract the topic name for microphone input and set the audio sample rate
    std::string topicName = extractTopic("Microphone");
    checkTopicAvailable(topicName);
    int sampleRate = 48000;

    // Check if the topic name is empty
    if (topicName.empty()) {
        ROS_WARN_STREAM("No valid topic found for Microphone. Skipping this sensor test.");
        return;
    }

    ROS_INFO_STREAM("Start " << topicName << " Subscribe Test\n");
    ros::Duration(1).sleep();

    // Open the output audio file for writing in binary mode
    #ifdef ROS
        outAudio.open(ros::package::getPath(ROS_PACKAGE_NAME) + "/data/microphoneOutput.wav", std::ios::binary);
    #else
        ROS_INFO_STREAM("Unable to open the output file microphoneOutput.wav\n");
        promptAndExit(1);
    #endif

    // Subscribe to the microphone topic with a larger queue size for audio data
    ros::Subscriber sub = nh.subscribe(topicName, 1000, microphoneMessageReceived);
    
    // Record audio from four different microphone channels sequentially
    for (int i = 0; i < 4; ++i) { // Four channels to record

        ROS_INFO_STREAM("Recording channel " << currentChannel << "\n");
        
        // Record audio from the current channel for the specified time duration
        ros::Time startTime = ros::Time::now(); // Record the start time
        ros::Time endTime = startTime + ros::Duration(timeDuration); // Calculate the end time

        while (ros::ok() && ros::Time::now() < endTime) {
            ros::spinOnce();
        }

        switchMicrophoneChannel(); // Switch to the next channel for recording
    }

    // Write the WAV file header with the recorded audio information
    outAudio.seekp(0, std::ios::beg);
    writeWavHeader(outAudio, sampleRate, totalSamples);

    // Close the output file and finish the microphone test
    outAudio.close();
    ROS_INFO_STREAM("Microphone test finished\n");

    // Play the recorded audio file and clean up
    playAndDeleteFile();
}

void microphoneMessageReceived(const naoqi_driver::AudioCustomMsg& msg) {
    /*
     * Callback function to process received microphone audio messages
     * The function extracts audio data from the specified channel and writes it to the output file
     *
     * @param:
     *     msg: The received audio message containing data from multiple microphone channels
     *
     * @return:
     *     None
     */
    
    // Check if the current channel is valid and the output file is open
    if (currentChannel.empty() || !outAudio.is_open()) {
        return;
    }

    // Pointer to hold the audio data from the selected channel
    const std::vector<int16_t>* channelData = nullptr;

    // Select the appropriate channel data based on the current channel setting
    if (currentChannel == "frontLeft") {
        channelData = &msg.frontLeft;
    } else if (currentChannel == "frontRight") {
        channelData = &msg.frontRight;
    } else if (currentChannel == "rearLeft") {
        channelData = &msg.rearLeft;
    } else if (currentChannel == "rearRight") {
        channelData = &msg.rearRight;
    }

    // Write the audio samples from the selected channel to the output file
    if (channelData) {
        for (const auto& sample : *channelData) {
            outAudio.write(reinterpret_cast<const char*>(&sample), sizeof(sample));
            totalSamples++; // Keep track of the total number of samples written
        }
    }
}

void stereoCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received stereo camera image messages
     * The function processes stereo camera images, displays them, saves video output, and logs image information
     *
     * @param:
     *     msg: The received image message containing stereo camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath, videoPath, writerKey = "stereo_camera";
    static bool pathReady = false;

    if (!pathReady) {
    #ifdef ROS
        basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    #else
        ROS_INFO_STREAM("Unable to find the ROS package");
        promptAndExit(1);
    #endif
        videoPath = basePath + "/data/stereoCameraOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] Stereo Image W:%d H:%d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) { printf("Unable to open %s\n", outputFilePath.c_str()); promptAndExit(1); }
        outputFile << "[TESTING] ---- STEREO CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES]\n\n";
        outputFile.close();
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, 10.0, true);
    if (w.opened) w.writer.write(img);

    cv::imshow("Stereo Camera", img);
    cv::waitKey(1);
}    
#endif

void jointStateMessageReceived(const sensor_msgs::JointState& msg) {
    /*
     * Callback function to process received joint state messages from the robot
     * The function prints joint information to console and optionally writes data to an output file
     *
     * @param:
     *     msg: Joint state message containing joint names, positions, velocities, and efforts
     *
     * @return:
     *     None
     */
    
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing joint state data received.\n");
    // Print the received message attributes to console
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

    // Extract the ROS package path for file operations    
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the received message data to an output file if the output flag is true
    if (output == true){
        // Construct the complete path for the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // Write joint state data to the output file
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
        
        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void odomMessageReceived(const nav_msgs::Odometry& msg){
    /*
     * Callback function to process received odometry messages from the robot
     * The function prints odometry data to console and optionally writes data to an output file
     *
     * @param:
     *     msg: Odometry message containing pose and twist information
     *
     * @return:
     *     None
     */
    
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing odometry data received.\n");
    // Print the received odometry message attributes to console
    ROS_INFO_STREAM("Header: " << msg.header << "\n" );
    ROS_INFO_STREAM("Child frame id: " << msg.child_frame_id << "\n" );
    ROS_INFO_STREAM("Pose: " << msg.pose << "\n" );
    ROS_INFO_STREAM("Twist: " << msg.twist << "\n" );
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");
    
    // Extract the ROS package path for file operations
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif
        
    // Write the received message data to an output file if the output flag is true
    if (output == true){
        
        // Construct the complete path for the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // Write odometry data to the output file
        outputFile << "[TESTING] ---- ODOMETRY ----\n\n";
        outputFile << "[MESSAGES] Printing odometry data received.\n";
        outputFile << "Header: " << msg.header << "\n" ;
        outputFile << "Child frame id: " << msg.child_frame_id << "\n" ;
        outputFile << "Pose: " << msg.pose << "\n" ;
        outputFile << "Twist: " << msg.twist << "\n" ;
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void imuMessageReceived(const sensor_msgs::Imu& msg) {
    /*
     * Callback function to process received IMU messages from the robot
     * The function prints IMU data to console and optionally writes data to an output file
     *
     * @param:
     *     msg: IMU message containing orientation, angular velocity, and linear acceleration data
     *
     * @return:
     *     None
     */
    
    string path;

    ROS_INFO_STREAM("[MESSAGES] Printing IMU data received.\n");
    // Print the received IMU message attributes to console
    ROS_INFO_STREAM("Header: " << msg.header << "\n" );
    ROS_INFO_STREAM("Orientation: " << msg.orientation << "\n" );
    ROS_INFO_STREAM("Angular velocity: " << msg.angular_velocity << "\n" );
    ROS_INFO_STREAM("Linear acceleration: " << msg.linear_acceleration << "\n" );
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

    // Extract the ROS package path for file operations
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif
    
    // Write the received message data to an output file if the output flag is true
    if (output == true){
        
        // Construct the complete path for the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }

        // Write IMU data to the output file
        outputFile << "[TESTING] ---- IMU ----\n\n";
        outputFile << "[MESSAGES] Printing IMU data received.\n";
        outputFile << "Header: " << msg.header << "\n" ;
        outputFile << "Orientation: " << msg.orientation << "\n" ;
        outputFile << "Angular velocity: " << msg.angular_velocity << "\n" ;
        outputFile << "Linear acceleration: " << msg.linear_acceleration << "\n" ;
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void backSonarMessageReceived(const sensor_msgs::Range& msg) {
    /*
     * Callback function to process received back sonar sensor messages
     * The function prints sonar range data to console and optionally writes data to an output file
     *
     * @param:
     *     msg: Range message containing sonar sensor data including range values and sensor parameters
     *
     * @return:
     *     None
     */
    
    string path;

    // Print a message indicating that sonar data is being processed
    ROS_INFO_STREAM("[MESSAGES] Printing back sonar data received.\n");

    // Print the received sonar message attributes to console
    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );          // Print the frame ID of the received message
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" );       // Print the field of view of the sonar sensor
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" );     // Print the minimum range value reported by the sonar sensor
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" );     // Print the maximum range value reported by the sonar sensor
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" );                 // Print the received range value reported by the sonar sensor
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");                 // Print a message indicating the end of printing sonar data
    
    // Extract the ROS package path for file operations
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the received message data to an output file if the output flag is true
    if (output == true){
        // Construct the complete path for the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }
        
        // Write back sonar data to the output file
        outputFile << "[TESTING] ---- BACK SONAR ----\n\n";
        outputFile << "[MESSAGES] Printing back sonar data received.\n";
        outputFile << "Frame id: " << msg.header.frame_id << "\n";
        outputFile << "Field of view: " << msg.field_of_view << "\n";
        outputFile << "Minimum range value: " << msg.min_range << "\n";
        outputFile << "Maximum range value: " << msg.max_range << "\n";
        outputFile << "Range value: " << msg.range << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void frontSonarMessageReceived(const sensor_msgs::Range& msg) {
    /*
     * Callback function to process received front sonar sensor messages
     * The function prints sonar range data to console and optionally writes data to an output file
     *
     * @param:
     *     msg: Range message containing sonar sensor data including range values and sensor parameters
     *
     * @return:
     *     None
     */
    
    string path;

    // Print a message indicating that front sonar data is being processed
    ROS_INFO_STREAM("[MESSAGES] Printing front sonar data received.\n");

    // Print the received front sonar message attributes to console
    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );          
    ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" );       
    ROS_INFO_STREAM("Minimum range value: " << msg.min_range << "\n" );     
    ROS_INFO_STREAM("Maximum range value: " << msg.max_range << "\n" );     
    ROS_INFO_STREAM("Range value: " << msg.range << "\n" );                 
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");                 

    // Extract the ROS package path for file operations
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the received message data to an output file if the output flag is true
    if (output == true){
        // Construct the complete path for the output file
        outputFilePath = path + "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(outputFilePath.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }
        
        // Write front sonar data to the output file
        outputFile << "[TESTING] ---- FRONT SONAR ----\n\n";
        outputFile << "[MESSAGES] Printing front sonar data received.\n";
        outputFile << "Frame id: " << msg.header.frame_id << "\n";
        outputFile << "Field of view: " << msg.field_of_view << "\n";
        outputFile << "Minimum range value: " << msg.min_range << "\n";
        outputFile << "Maximum range value: " << msg.max_range << "\n";
        outputFile << "Range value: " << msg.range << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";

        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received front camera image messages
     * The function processes camera images, displays them, saves video output, and logs image information
     *
     * @param:
     *     msg: Image message containing front camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath, videoPath, writerKey = "front_camera";
    static bool pathReady = false;

    if (!pathReady) {
    #ifdef ROS
        basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    #else
        ROS_INFO_STREAM("Unable to find the ROS package");
        promptAndExit(1);
    #endif
        videoPath = basePath + "/data/frontCameraOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] Front Image received W:%d H:%d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) { printf("Unable to open %s\n", outputFilePath.c_str()); promptAndExit(1); }
        outputFile << "[TESTING] ---- FRONT CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES]\n\n";
        outputFile.close();
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, /*fps=*/10.0, /*isColor=*/true);
    if (w.opened) w.writer.write(img);

    cv::imshow("Front Camera", img);
    cv::waitKey(1);
}

void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received bottom camera image messages
     * The function processes camera images, displays them, saves video output, and logs image information
     *
     * @param:
     *     msg: Image message containing bottom camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath, videoPath, writerKey = "bottom_camera";
    static bool pathReady = false;

    if (!pathReady) {
    #ifdef ROS
        basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    #else
        ROS_INFO_STREAM("Unable to find the ROS package");
        promptAndExit(1);
    #endif
        videoPath = basePath + "/data/bottomCameraOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] Bottom Image received W:%d H:%d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) { printf("Unable to open %s\n", outputFilePath.c_str()); promptAndExit(1); }
        outputFile << "[TESTING] ---- BOTTOM CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES]\n\n";
        outputFile.close();
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, 10.0, true);
    if (w.opened) w.writer.write(img);

    cv::imshow("Bottom Camera", img);
    cv::waitKey(1);
}

void realsenseRGBCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received RealSense RGB camera image messages
     * The function processes RGB camera images, displays them, saves video output, and logs image information
     *
     * @param:
     *     msg: Image message containing RealSense RGB camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath;
    static std::string videoPath;
    static std::string writerKey = "realsense_rgb_camera";
    static bool pathReady = false;

    if (!pathReady) {
        #ifdef ROS
            basePath = ros::package::getPath(ROS_PACKAGE_NAME);
        #else
            ROS_INFO_STREAM("Unable to find the ROS package\n");
            promptAndExit(1);
        #endif
        videoPath = basePath + "/data/realsenseRGBOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] RealSense RGB Image received has a width: %d and height: %d", imgWidth, imgHeight);

    // One-time “info to file” block (unchanged)
    if (output == true) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            printf("Unable to open the output file %s\n", outputFilePath.c_str());
            promptAndExit(1);
        }
        outputFile << "[TESTING] ---- REALSENSE RGB CAMERA ----\n\n";
        outputFile << "[MESSAGES] Printing RealSense RGB camera data information received.\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "Encoding: " << msg->encoding << "\n";
        outputFile << "Step: " << msg->step << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();
        output = false;
    }

    // Convert to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    // Get the per-stream writer and write the frame
    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, /*fps=*/10.0, /*isColor=*/true);
    if (w.opened) w.writer.write(img);

    // Show the image
    cv::imshow("RealSense RGB Camera", img);
    cv::waitKey(1);  // keep UI responsive; 1ms is fine
}

void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received depth camera image messages
     * The function processes depth images, converts them to displayable format, saves video output, and logs image information
     *
     * @param:
     *     msg: Image message containing depth camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath, videoPath, writerKey = "depth_camera";
    static bool pathReady = false;

    if (!pathReady) {
    #ifdef ROS
        basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    #else
        ROS_INFO_STREAM("Unable to find the ROS package");
        promptAndExit(1);
    #endif
        videoPath = basePath + "/data/depthCameraOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] Depth Image received W:%d H:%d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) { printf("Unable to open %s\n", outputFilePath.c_str()); promptAndExit(1); }
        outputFile << "[TESTING] ---- DEPTH CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES]\n\n";
        outputFile.close();
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth = cv_ptr->image;               // 16U, millimeters expected
    cv::Mat viz8u, color;

    // Scale 0..1000mm -> 0..255 for visualization
    depth.convertTo(viz8u, CV_8UC1, 255.0 / 1000.0);
    cv::applyColorMap(viz8u, color, cv::COLORMAP_JET);

    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, 10.0, /*isColor=*/true);
    if (w.opened) w.writer.write(color);

    cv::imshow("Depth Camera", color);
    cv::waitKey(1);
}

void realsenseDepthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg) {
    /*
     * Callback function to process received RealSense depth camera image messages
     * The function processes depth images, converts them to displayable format, saves video output, and logs image information
     *
     * @param:
     *     msg: Image message containing RealSense depth camera data
     *
     * @return:
     *     None
     */
    
    // Variables to store file paths for output
    static std::string basePath, videoPath, writerKey = "realsense_depth_camera";
    static bool pathReady = false;

    if (!pathReady) {
    #ifdef ROS
        basePath = ros::package::getPath(ROS_PACKAGE_NAME);
    #else
        ROS_INFO_STREAM("Unable to find the ROS package");
        promptAndExit(1);
    #endif
        videoPath = basePath + "/data/realsenseDepthOutput.mp4";
        pathReady = true;
    }

    const int imgWidth  = msg->width;
    const int imgHeight = msg->height;
    ROS_INFO("[MESSAGE] RealSense Depth Image W:%d H:%d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = basePath + "/data/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) { printf("Unable to open %s\n", outputFilePath.c_str()); promptAndExit(1); }
        outputFile << "[TESTING] ---- REALSENSE DEPTH CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "Encoding: " << msg->encoding << "\n";
        outputFile << "Step: " << msg->step << "\n";
        outputFile << "[END MESSAGES]\n\n";
        outputFile.close();
        output = false;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } else {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth = cv_ptr->image;
    cv::Mat viz8u, color;

    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        // assume 0..5000 mm range
        depth.convertTo(viz8u, CV_8UC1, 255.0 / 5000.0);
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        // assume 0..5.0 m range
        depth.convertTo(viz8u, CV_8UC1, 255.0 / 5.0);
    } else {
        if (depth.channels() == 1) {
            cv::normalize(depth, viz8u, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        } else {
            viz8u = depth.clone();
        }
    }

    cv::applyColorMap(viz8u, color, cv::COLORMAP_JET);

    auto& w = getOrOpenWriter(writerKey, videoPath, imgWidth, imgHeight, 10.0, true);
    if (w.opened) w.writer.write(color);

    cv::imshow("RealSense Depth Camera", color);
    cv::waitKey(1);
}

void laserSensorMessageReceived(const sensor_msgs::LaserScan& msg) {
    /*
     * Callback function to process received laser scan messages
     * The function prints laser scan data to console and optionally writes data to an output file
     *
     * @param:
     *     msg: LaserScan message containing laser sensor data including ranges and scan parameters
     *
     * @return:
     *     None
     */
    
    string path;
    
    ROS_INFO_STREAM("[MESSAGES] Printing laser sensor data received.\n");
    // Print the received laser scan message attributes to console
    ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );
    ROS_INFO_STREAM("Start angle of the scan: " << msg.angle_min << "\n" );
    ROS_INFO_STREAM("End angle of the scan: " << msg.angle_max << "\n" );
    ROS_INFO_STREAM("Angular distance between measurements: " << msg.angle_increment << "\n" );
    ROS_INFO_STREAM("Time between measurements: " << msg.time_increment << "\n" );
    ROS_INFO_STREAM("Time between scans: " << msg.scan_time << "\n" );
    ROS_INFO_STREAM("Minimum range value: " << msg.range_min << "\n" );
    ROS_INFO_STREAM("Maximum range value: " << msg.range_max << "\n" );
    ROS_INFO_STREAM("Range data: (size: " << msg.ranges.size() << ") \n" );
            
    // Print all range values from the laser scan
    for (auto rng : msg.ranges){
        ROS_INFO_STREAM(rng);
    }
    ROS_INFO_STREAM("\n");
    ROS_INFO_STREAM("[END MESSAGES] Finished printing.\n");

    // Extract the ROS package path for file operations
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        ROS_INFO_STREAM("Unable to find the ROS package\n");
        promptAndExit(1);
    #endif

    // Write the received message data to an output file if the output flag is true
    if (output == true){
        // Construct the complete path for the output file
        path += "/data/sensorTestOutput.dat";
        
        // Open the output file in append mode
        outputFile.open(path.c_str(), ofstream::app);
        if (!outputFile.is_open()){
            printf("Unable to open the output file %s\n", path.c_str());
            promptAndExit(1);
        }

        // Write laser sensor data to the output file
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
        
        // Close the output file and set flag to prevent repeated writing
        outputFile.close();
        output = false;
    }
}

void promptAndExit(int status){
    /*
     * Helper function to prompt the user for input before exiting the program
     * The function displays a message and waits for user input before terminating with the specified status
     *
     * @param:
     *     status: Exit status code to be used when terminating the program
     *
     * @return:
     *     None (function terminates the program)
     */
    
    printf("Press any key to continue ... \n");
    getchar(); // Wait for user input
    exit(status); // Terminate the program with the specified status code
}

void promptAndContinue(){
    /*
     * Helper function to prompt the user for input before continuing program execution
     * The function displays a message and waits for user input before returning control to the caller
     *
     * @param:
     *     None
     *
     * @return:
     *     None
     */
    
    printf("Press any key to proceed ...\n");
    getchar(); // Wait for user input before continuing
}

string extractTopic(string key){
    /*
     * Function to extract topic names from configuration files for sensor testing
     * The function reads configuration files to determine the platform and retrieves appropriate topic names
     *
     * @param:
     *     key: The sensor/topic key to search for in the configuration files
     *
     * @return:
     *     string: The topic name corresponding to the provided key, or empty string if not found
     */
    
    // Initialize debug flag and configuration file variables
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

    // Construct the full path of the configuration file using ROS package path
    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
        promptAndExit(1);
    #endif

    // Set configuration file path
    configPathFile = packagePath + "/config/" + configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open and read the configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Parse key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        
        // Store configuration values based on the parameter key
        if (paramKey == platformKey){ platformValue = paramValue;}
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}
        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}
    }
    
    configFile.close();

    // Determine the topic file based on the platform configuration
    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topicFileName.c_str());

    // Construct the full path to the topic file
    topicPathFile = packagePath + "/data/" + topicFileName;

    if (debug) printf("Topic file is %s\n", topicPathFile.c_str());

    // Open and read the topic file
    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()){
        printf("Unable to open the topic file %s\n", topicPathFile.c_str());
        promptAndExit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Search for the specified key in the topic file
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
            break; // Found the topic, exit the loop
        }
    }
    topicFile.close();

    return topic_value;
}

std::vector<std::string> extractTests(const std::string& cameraSel) {
    /*
    * @param:
    *     camera: Camera selection string from the node parameter (e.g., "~camera").
    *             Expected values are "pepper", "realsense", or "both" (case-insensitive).
    *
    * @return:
    *     std::vector<std::string>: Vector of enabled test names (lowercased) after applying
    *     camera-based filtering. Order preserves the appearance order in the input file
    *     minus any removed entries.
    *
    * @note:
    *     - Relies on ROS package path resolution via ros::package::getPath(ROS_PACKAGE_NAME).
    *     - Trims surrounding whitespace and lowercases both keys and boolean values.
    *     - Lines beginning with '#' or empty lines are ignored.
    *     - If the configuration file cannot be found or opened, the function prints an
    *       error message and terminates via promptAndExit(1).
    */

    bool debug = false;

    std::string inputFileName = "sensorTestInput.dat";
    std::string inputPath;
    std::string inputPathFile;

    std::vector<std::string> testName;
    std::string flag;

    #ifdef ROS
        inputPath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define it.\n");
        promptAndExit(1);
    #endif

    inputPathFile = inputPath + "/data/" + inputFileName;
    if (debug) printf("Input file is %s\n", inputPathFile.c_str());

    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()){
        printf("Unable to open the input file %s\n", inputPathFile.c_str());
        promptAndExit(1);
    }

    std::string line, key, value;
    while (std::getline(inputFile, line)) {
        if (line.empty() || line[0] == '#') continue;      // skip blanks/comments
        std::istringstream iss(line);
        if (!(iss >> key >> value)) continue;              // expect "key  true/false"

        trim(key);
        trim(value);
        key   = toLower(key);
        value = toLower(value);

        if (value == "true") {
            testName.push_back(key);
        }
    }
    inputFile.close();

    // --- Camera-based filtering -----------------------------
    // Use lowercase names since we lowercased keys above
    const std::unordered_set<std::string> pepperCamTests = {
        "frontcamera", "bottomcamera", "depthcamera", "stereocamera"
    };
    const std::unordered_set<std::string> realsenseCamTests = {
        "realsensergbcamera", "realsensedepthcamera"
    };

    const std::string cam = toLower(cameraSel);
    auto drop_if_in = [&](const std::unordered_set<std::string>& toDrop){
        testName.erase(std::remove_if(testName.begin(), testName.end(),
            [&](const std::string& name){ return toDrop.count(name) > 0; }),
            testName.end());
    };

    if (cam == "pepper") {
        // keep Pepper cameras, drop RealSense
        drop_if_in(realsenseCamTests);
    } else if (cam == "realsense") {
        // keep RealSense, drop Pepper cameras
        drop_if_in(pepperCamTests);
    } // "both" → no filtering

    // ---------------------------------------------------------

    std::cout << "Tests to be executed: ";
    for (const auto& name : testName) std::cout << name << " ";
    std::cout << std::endl;

    return testName;
}

std::string extractMode(){
    /*
     * Function to extract the test execution mode from the configuration file
     * The function reads the configuration file and returns the specified test mode
     *
     * @param:
     *     None
     *
     * @return:
     *     std::string: The test execution mode (e.g., "sequential", "parallel")
     */
    
    // Initialize debug flag and configuration file variables
    bool debug = false;   // used to turn debug message on

    std::string configFileName = "sensorTestConfiguration.ini";     // configuration filename
    std::string packagePath;                                        // ROS package path
    std::string configPathFile;                                     // configuration path and filename

    std::string modeKey        = "mode";                            // mode key

    std::string modeValue;                                          // mode value

    // Construct the full path of the configuration file using ROS package path
    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
    #endif

    // Set configuration file path
    configPathFile = packagePath + "/config/" + configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    // Open and read the configuration file
    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()){
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    
    // Parse key-value pairs from the configuration file
    while(std::getline(configFile, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        // Convert to lower case for case-insensitive comparison
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        // Store the mode value if the key matches
        if (paramKey == modeKey){ modeValue = paramValue;}
    }
    configFile.close();

    // Verify that a valid mode value was found
    if (modeValue == ""){
        printf("Unable to find a valid mode.\n");
        promptAndExit(1);
    }
    return modeValue;
}

void switchMicrophoneChannel() {
    /*
     * Function to switch to the next microphone channel for sequential recording
     * The function cycles through four microphone channels: rearLeft -> rearRight -> frontLeft -> frontRight
     *
     * @param:
     *     None (operates on global variable currentChannel)
     *
     * @return:
     *     None
     */
    
    // Switch to the next microphone channel in sequence
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

void writeWavHeader(std::ofstream& file, int sampleRate, int numSamples) {
    /*
     * Function to write a standard WAV file header to an output stream
     * The function writes the necessary WAV format headers for 16-bit mono PCM audio
     *
     * @param:
     *     file: Output file stream to write the WAV header to
     *     sampleRate: Audio sample rate in Hz (e.g., 44100, 48000)
     *     numSamples: Total number of audio samples in the file
     *
     * @return:
     *     None
     */
    
    // Calculate WAV format parameters
    int byteRate = sampleRate * 2; // 16-bit mono = 2 bytes per sample
    int dataSize = numSamples * 2; // Total number of bytes in data
    int chunkSize = 36 + dataSize;
    
    // Write WAV file header components
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
    /*
     * Function to play the recorded microphone audio file and optionally delete it
     * The function uses the system's audio player to play the WAV file and can delete it afterward
     *
     * @param:
     *     None (operates on hardcoded file path)
     *
     * @return:
     *     None
     */
    
    // Configuration flag to control file deletion after playing    
    bool deleteFile = false;
    
    // Construct the full path to the recorded audio file
    std::string fileName = ros::package::getPath(ROS_PACKAGE_NAME) + "/data/microphoneOutput.wav";

    // Check if the audio file exists before attempting to play it
    std::ifstream file(fileName);
    if (!file.good()) {
        std::cerr << "Error: File not found: " << fileName << std::endl;
        return;
    }

    // Play the audio file using the system's audio player
    if (std::system(("aplay " + fileName).c_str()) != 0) {
        std::cerr << "Error playing file: " << fileName << std::endl;
        return; // Exit if playing failed
    }

    // Optionally delete the file after playing
    if (deleteFile) {
        if (std::remove(fileName.c_str()) != 0) {
            std::cerr << "Error deleting file: " << fileName << std::endl;
        } else {
            std::cout << "File deleted successfully: " << fileName << std::endl;
        }
    }
}

void initializeOutputFile(std::ofstream& outputFile, const std::string& path) {
    /*
     * Function to initialize the sensor test output file with header information
     * The function creates or appends to the output file and writes initial test information
     *
     * @param:
     *     outputFile: Reference to the output file stream
     *     path: Full path to the output file
     *
     * @return:
     *     None
     */
    
    // Open the output file in append mode
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        std::cerr << "Unable to open the output file " << path << "\n";
        throw std::runtime_error("Failed to open output file.");
    }

    // Write test session header and start time
    outputFile << "[TESTING] ############ SENSORS ############\n\n";
    outputFile << "[START TIME] " << getCurrentTime() << "\n";
    
    outputFile.close();
}

void finalizeOutputFile(std::ofstream& outputFile, const std::string& path) {
    /*
     * Function to finalize the sensor test output file with closing information
     * The function appends the test end time to mark the completion of the test session
     *
     * @param:
     *     outputFile: Reference to the output file stream
     *     path: Full path to the output file
     *
     * @return:
     *     None
     */
    
    // Open the output file in append mode and write end time
    outputFile.open(path, std::ofstream::app);
    outputFile << "[END TIME] " << getCurrentTime() << "\n\n";
    outputFile.close();
}

std::string getCurrentTime() {
    /*
     * Function to get the current system time as a formatted string
     * The function returns the current date and time in a standardized format
     *
     * @param:
     *     None
     *
     * @return:
     *     std::string: Formatted current date and time string
     */
    
    // Format the current time into a readable string
    char buffer[50];
    std::time_t now = std::time(0);
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d.%X", std::localtime(&now));
    return std::string(buffer);
}

std::string getOutputFilePath() {
    /*
     * Function to construct the full path for the sensor test output file
     * The function builds the output file path using the ROS package path and predefined filename
     *
     * @param:
     *     None
     *
     * @return:
     *     std::string: Complete path to the output file
     */
    
    std::string basePath;
    std::string fileName = "sensorTestOutput.dat";

    // Construct the output file path using ROS package path
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

bool checkTopicAvailable(const std::string& topic_name_raw) {
    /*
    * @brief
    *     Checks availability of a specified ROS topic or action base in a single shot.
    *
    * @param:
    *     topic_name_raw: Raw name of the topic or action base to check.
    *
    * @return:
    *     true if the topic exists or if the action base has at least one standard subtopic,
    *     false otherwise.
    *
    * @note:
    *     Prints an informational message with the current node name when a subscription
    *     is detected. Performs no retries or waiting — call repeatedly if you need
    *     continuous checking.
    */

    if (!ros::master::check()) return false;

    const std::string resolved = rstrip_slash(ros::names::resolve(topic_name_raw));
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) return false;

    // Helper to strip leading slash
    auto cleanNodeName = [](const std::string& s) {
        return (!s.empty() && s.front() == '/') ? s.substr(1) : s;
    };

    const std::string nodeName = cleanNodeName(ros::this_node::getName());

    // Fast path: exact match
    for (const auto& t : topics) {
        if (t.name == resolved) {
            ROS_INFO_STREAM(nodeName << ": subscribed to " << resolved << ".");
            return true;
        }
    }

    // Action-style check
    static const char* kActionSuffixes[] = {
        "/goal", "/status", "/feedback", "/result", "/cancel"
    };
    for (const auto& t : topics) {
        for (const char* suf : kActionSuffixes) {
            if (t.name == resolved + suf) {
                ROS_INFO_STREAM(nodeName << ": subscribed to " << resolved << ".");
                return true;
            }
        }
    }

    return false;
}

void executeTestsSequentially(const std::vector<std::string>& testNames, ros::NodeHandle& nh) {
    /*
     * Function to execute sensor tests sequentially one after another
     * The function maps test names to their corresponding functions and executes them in order
     *
     * @param:
     *     testNames: Vector of test names to be executed
     *     nh: ROS node handle for sensor communication
     *
     * @return:
     *     None
     */
    
    // Map test names to their corresponding sensor test functions
    std::unordered_map<std::string, TestFunction> testMap = {
        {"backsonar", backSonar},
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera},
        {"bottomcamera", bottomCamera},
        {"realsensergbcamera", realsenseRGBCamera},
        {"realsensedepthcamera", realsenseDepthCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu}
    };

    // Add Pepper robot specific tests if compiled for Pepper
    #ifdef PEPPER_ROBOT
    testMap["stereocamera"] = stereoCamera;
    testMap["microphone"] = microphone;
    testMap["speech"] = speech;
    #endif

    // Execute each test sequentially
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
    /*
     * Function to execute sensor tests in parallel using multiple threads
     * The function creates separate threads for each test to run them simultaneously
     *
     * @param:
     *     testNames: Vector of test names to be executed
     *     nh: ROS node handle for sensor communication
     *
     * @return:
     *     None
     */
    
    // Map test names to their corresponding sensor test functions
    std::unordered_map<std::string, TestFunction> testMap = {
        {"backsonar", backSonar},
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera}, 
        {"realsensergbcamera", realsenseRGBCamera},
        {"realsensedepthcamera", realsenseDepthCamera},  
        {"bottomcamera", bottomCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu},
    };

    // Add Pepper robot specific tests if compiled for Pepper
    #ifdef PEPPER_ROBOT
    testMap["stereocamera"] = stereoCamera;
    testMap["microphone"] = microphone;
    testMap["speech"] = speech;
    #endif

    std::vector<std::thread> threads;

    // Create a separate thread for each test found in the map
    for (const auto& testName : testNames) {
        auto it = testMap.find(testName);
        if (it != testMap.end()) {
            threads.emplace_back(it->second, std::ref(nh));
        } else {
            std::cerr << "There is no test function associated with the test name: " << testName << "Proceeding to the next test...\n";
        }
    }

    // Wait for all threads to finish execution before continuing
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}