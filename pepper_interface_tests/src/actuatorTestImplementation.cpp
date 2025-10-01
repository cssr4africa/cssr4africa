/* actuatorTestImplementation.cpp Implementation code for running the actuator tests on Pepper robot
*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa 
* Date: September 25, 2025
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

/*  Description:
* This file contains the implementation of the actuator tests for the Pepper robot.  The tests are designed 
* to test the head, arms, hands, legs and wheels of the robot. The tests are implemented using the ROS actionlib 
* library.The test will move the robot's head, arms, hands, legs and wheels to the minimum position, then to the 
* maximum position, then to the mid-range position. 

* For the wheels, the tests will publish a position on the cmd_vel topic to move the robot forward, backward and 
* do 90 degree turns both clockwise and counter-clockwise.
*/

# include "pepper_interface_tests/actuatorTestInterface.h"

// Global variables for the wheels
bool verboseMode = false; 
bool shutdownInitiated = false;
ros::Time startTime;
ros::Publisher pub;

enum Robotstate{
    MOVE_FORWARD,
    MOVE_BACKWARD,
    ROTATE_CLOCKWISE,
    ROTATE_COUNTER_CLOCKWISE,
    STOP
};

Robotstate state = MOVE_FORWARD;

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

void signalHandler(int signum) {
    /*
     * Signal handler to safely stop the robot when interrupt signal is received
     * Publishes zero velocities multiple times to ensure the robot stops completely
     *
     * @param:
     *     signum: Signal number received (typically SIGINT for Ctrl+C)
     *
     * @return:
     *     None (terminates the program)
     */
    
    ROS_WARN("Interrupt signal (%d) received. Stopping the robot.", signum);

    geometry_msgs::Twist stopMsg;
    stopMsg.linear.x = 0.0;
    stopMsg.angular.z = 0.0;
    
    ros::Rate rate(10);  
    for (int i = 0; i < 30; ++i) {
        pub.publish(stopMsg);
        rate.sleep();
    }

    ros::shutdown();
}

ControlClientPtr createClient(const std::string& topicName) {
    /*
     * Creates and initializes an action client for joint trajectory control
     * Waits for the action server to become available with retry mechanism
     *
     * @param:
     *     topicName: Name of the ROS topic for the action server
     *
     * @return:
     *     ControlClientPtr: Shared pointer to the initialized action client
     *
     * @throws:
     *     std::runtime_error: If action server is not available after maximum iterations
     */
    
    ControlClientPtr actionClient(new ControlClient(topicName, true));
    int maxIterations = 5;

    for (int iterations = 0; iterations < maxIterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topicName.c_str());
    }

    throw std::runtime_error("Error creating action client for " + topicName + " controller: Server not available");
}

void moveToPosition(ControlClientPtr& client, const std::vector<std::string>& jointNames, double duration, 
                    const std::string& positionName, std::vector<double> positions) {
    /*
     * Moves robot joints to specified positions using trajectory control
     * Sends joint trajectory goal and waits for completion with status reporting
     *
     * @param:
     *     client: Shared pointer to the action client for joint control
     *     jointNames: Vector of joint names to control
     *     duration: Time duration for the movement in seconds
     *     positionName: Descriptive name for the target position (for logging)
     *     positions: Target positions for each joint in radians
     *
     * @return:
     *     None
     */
    
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = jointNames;
    trajectory.points.resize(1);

    trajectory.points[0].positions = positions;
    trajectory.points[0].time_from_start = ros::Duration(duration);

    client->sendGoal(goal);

    bool finishedBeforeTimeout = client->waitForResult(ros::Duration(10.0));

    if (finishedBeforeTimeout) {
        actionlib::SimpleClientGoalState state = client->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully moved to %s position.", positionName.c_str());
        } else {
            ROS_WARN("The action failed to move to %s position. State: %s", positionName.c_str(), state.toString().c_str());
        }
    } else {
        ROS_WARN("The action did not finish before the timeout.");
    }
}

std::vector<std::vector<double>> calculateDuration(std::vector<double> homePosition, std::vector<double> maxPosition, 
                                                  std::vector<double> minPosition, std::vector<std::vector<double>> velocity) {
    /*
     * Calculates movement durations for joint trajectories based on position differences and velocities
     * Computes time required for three-phase movement: home->min, min->max, max->home
     *
     * @param:
     *     homePosition: Vector of home/rest positions for each joint
     *     maxPosition: Vector of maximum positions for each joint
     *     minPosition: Vector of minimum positions for each joint
     *     velocity: 2D vector of velocities for each joint and movement phase
     *
     * @return:
     *     std::vector<std::vector<double>>: 2D vector containing calculated durations for each joint and phase
     */
    
    std::vector<std::vector<double>> duration(velocity.size(), std::vector<double>(velocity[0].size(), 0.0));
    
    for (int i = 0; i < homePosition.size(); ++i) {
        duration[i][0] = std::fabs(minPosition[i] - homePosition[i]) / velocity[i][0];
        duration[i][1] = std::fabs(maxPosition[i] - minPosition[i]) / velocity[i][1];
        duration[i][2] = std::fabs(homePosition[i] - maxPosition[i]) / velocity[i][2];   
    }

    return duration;
}

void head(ros::NodeHandle& nh) {
    /*
     * Performs comprehensive head movement testing for HeadPitch and HeadYaw joints
     * Tests full range of motion by moving each joint to min, max, and mid positions
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string headTopic = extractTopic("Head");
    checkTopicAvailable(headTopic);
    ControlClientPtr headClient = createClient(headTopic);
    std::vector<std::string> jointNames = {"HeadPitch", "HeadYaw"};
    std::vector<double> position(2, 0.0);
    
    std::vector<double> minPosition = {-0.706, -2.085};
    std::vector<double> maxPosition = {0.445, 2.085};
    std::vector<double> homePosition = {-0.2, 0.012};
    
    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 1.5},{1.2, 1.2, 1.2}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);
    
    ROS_INFO_STREAM("----------[START HEAD CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = minPosition[i];
        moveToPosition(headClient, jointNames, duration[i][0], "min", position);

        position[i] = maxPosition[i];
        moveToPosition(headClient, jointNames, duration[i][1], "max", position);

        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(headClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN HEAD] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(headClient, jointNames, homeDuration, "home", homePosition);

    ROS_INFO_STREAM("----------[END HEAD CONTROL TEST]-----------");
}

void rArm(ros::NodeHandle& nh) {
    /*
     * Performs comprehensive right arm movement testing for all shoulder, elbow, and wrist joints
     * Tests full range of motion by moving each joint through min, max, and mid positions
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string rightArmTopic = extractTopic("RArm");
    checkTopicAvailable(rightArmTopic);
    ControlClientPtr rightArmClient = createClient(rightArmTopic);
    std::vector<std::string> jointNames = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};
    std::vector<double> position(5, 0.0);
    
    std::vector<double> minPosition = {-2.0857, -1.5620 , 0.0087, -2.0857, -1.5620};
    std::vector<double> maxPosition = {2.0857,  -0.0087,  1.5620,  2.0857,  1.8239};
    std::vector<double> homePosition = {1.7410, -0.09664, 0.09664, 1.6981, -0.05679};
  
    std::vector<std::vector<double>> velocity = {{1.5, 1.5, 0.1}, {1.2, 0.8, 0.15},{0.1, 0.8, 1.2}, {2.0, 1.5, 0.2}, {1.8, 1.8, 1.8}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocity);

    ROS_INFO_STREAM("----------[START RIGHT ARM CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = minPosition[i];
        moveToPosition(rightArmClient, jointNames, duration[i][0], "min", position);

        position[i] = maxPosition[i];
        moveToPosition(rightArmClient, jointNames, duration[i][1], "max", position);

        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(rightArmClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN RIGHT ARM] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(rightArmClient, jointNames, homeDuration, "home", homePosition);

    ROS_INFO_STREAM("----------[END RIGHT ARM CONTROL TEST]-----------");
}

void rHand(ros::NodeHandle& nh) {
    /*
     * Performs right hand movement testing for hand opening and closing functionality
     * Tests the full range of hand motion from fully closed to fully open
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string rightHandTopic = extractTopic("RHand");
    checkTopicAvailable(rightHandTopic);
    ControlClientPtr rightHandClient = createClient(rightHandTopic);
    std::vector<std::string> jointNames = {"RHand"};
    std::vector<double> position(1, 0.0);
    
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.66608};
    double velocity = 2.0;

    double duration = std::fabs(maxPosition[0] - minPosition[0]) / velocity;

    ROS_INFO_STREAM("----------[START RIGHT HAND CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = minPosition[i];
        moveToPosition(rightHandClient, jointNames, duration, "min", position);

        position[i] = maxPosition[i];
        moveToPosition(rightHandClient, jointNames, duration, "max", position);

        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(rightHandClient, jointNames, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN RIGHT HAND] Moving to the Home position");
    moveToPosition(rightHandClient, jointNames, duration, "home", homePosition);

    ROS_INFO_STREAM("----------[END RIGHT HAND CONTROL TEST]-----------");
}

void lArm(ros::NodeHandle& nh) {
    /*
     * Performs comprehensive left arm movement testing for all shoulder, elbow, and wrist joints
     * Tests full range of motion by moving each joint through min, max, and mid positions
     * Note: Left arm has inverted motion patterns compared to right arm
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string leftArmTopic = extractTopic("LArm");
    checkTopicAvailable(leftArmTopic);
    ControlClientPtr leftArmClient = createClient(leftArmTopic);
    std::vector<std::string> jointNames = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
    std::vector<double> position(5, 0.0);
    
    std::vector<double> minPosition = {2.0857,  0.0087,  -1.5620, -2.0857,  -1.8239};
    std::vector<double> maxPosition = {-2.0857, 1.5620 , -0.0087,  2.0857,   1.8239};
    std::vector<double> homePosition = {1.7625, 0.09970, -0.1334, -1.7150,  0.06592};

    std::vector<std::vector<double>> velocities = {{1.5, 1.5, 0.1},{1.2, 0.8, 0.15},{0.1, 0.9, 1.2},{2.1, 1.5, 0.2},{1.8, 1.8, 1.9}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, minPosition, maxPosition, velocities);

    ROS_INFO_STREAM("----------[START LEFT ARM CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = maxPosition[i];
        moveToPosition(leftArmClient, jointNames, duration[i][0], "min", position);

        position[i] = minPosition[i];
        moveToPosition(leftArmClient, jointNames, duration[i][1], "max", position);

        position[i] = (minPosition[i] + maxPosition[i]) / 2.0;
        moveToPosition(leftArmClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN LEFT ARM] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(leftArmClient, jointNames, homeDuration, "home", homePosition);

    ROS_INFO_STREAM("----------[END LEFT ARM CONTROL TEST]-----------");
}

void lHand(ros::NodeHandle& nh) {
    /*
     * Performs left hand movement testing for hand opening and closing functionality
     * Tests the full range of hand motion from fully closed to fully open
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string leftHandTopic = extractTopic("LHand");
    checkTopicAvailable(leftHandTopic);
    ControlClientPtr leftHandClient = createClient(leftHandTopic);
    std::vector<std::string> jointNames = {"LHand"};
    std::vector<double> position(1, 0.0);
    
    std::vector<double> maxPosition = {1.0};
    std::vector<double> minPosition = {0.0};
    std::vector<double> homePosition = {0.6695};

    double velocity = 2.0;
    double duration = std::fabs(maxPosition[0] - minPosition[0]) / velocity;

    ROS_INFO_STREAM("----------[START LEFT HAND CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = minPosition[i];
        moveToPosition(leftHandClient, jointNames, duration, "min", position);

        position[i] = maxPosition[i];
        moveToPosition(leftHandClient, jointNames, duration, "max", position);

        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(leftHandClient, jointNames, duration, "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN LEFT HAND] Moving to the Home position");
    moveToPosition(leftHandClient, jointNames, duration, "home", homePosition);

    ROS_INFO_STREAM("----------[END LEFT HAND CONTROL TEST]-----------");
}

void leg(ros::NodeHandle& nh) {
    /*
     * Performs leg movement testing for hip and knee joints
     * Tests the range of motion for HipPitch, HipRoll, and KneePitch joints
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string legTopic = extractTopic("Leg");
    checkTopicAvailable(legTopic);
    ControlClientPtr legClient = createClient(legTopic);
    std::vector<std::string> jointNames = {"HipPitch", "HipRoll", "KneePitch"};
    std::vector<double> position(3, 0.0);
    
    std::vector<double> minPosition = {-1.0385, -0.5149 , -0.5149};
    std::vector<double> maxPosition = {1.0385,   0.5149,   0.5149};
    std::vector<double> homePosition = {-0.0107, -0.00766, 0.03221};

    std::vector<std::vector<double>> velocities = {{0.5, 0.5, 0.5},{0.5, 0.5, 0.5},{0.5, 0.5, 0.5}};
    std::vector<std::vector<double>> duration = calculateDuration(homePosition, maxPosition, minPosition, velocities);

    ROS_INFO_STREAM("----------[START LEG CONTROL TEST]-----------");

    for (int i = 0; i < jointNames.size(); ++i) {
        ROS_INFO_STREAM("[START] " << jointNames[i] << " test.");

        position[i] = minPosition[i];
        moveToPosition(legClient, jointNames, duration[i][0], "min", position);

        position[i] = maxPosition[i];
        moveToPosition(legClient, jointNames, duration[i][1], "max", position);

        position[i] = (maxPosition[i] + minPosition[i]) / 2.0;
        moveToPosition(legClient, jointNames, duration[i][2], "mid", position);

        ROS_INFO_STREAM("[END] " << jointNames[i] << " test.");
    }

    ROS_INFO_STREAM("[PUT DOWN LEG] Moving to the Home position");
    double homeDuration = 2.0;
    moveToPosition(legClient, jointNames, homeDuration, "home", homePosition);

    ROS_INFO_STREAM("----------[END LEG CONTROL TEST]-----------");
}

void wheels(ros::NodeHandle& nh) {
    /*
     * Performs comprehensive wheel movement testing using state machine
     * Tests forward movement, backward movement, clockwise rotation, and counter-clockwise rotation
     * Each movement state has a specific duration before transitioning to the next state
     *
     * @param:
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     */
    
    std::string wheelTopic = extractTopic("Wheels");
    checkTopicAvailable(wheelTopic);
    pub = nh.advertise<geometry_msgs::Twist>(wheelTopic, 1000);
    ros::Rate rate(10);

    signal(SIGINT, signalHandler);
    geometry_msgs::Twist msg;
    startTime = ros::Time::now();

    ROS_INFO_STREAM("----------[START WHEEL CONTROL TEST]-----------");

    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration elapsedTime = ros::Time::now() - startTime;

        switch (state) {
            case MOVE_FORWARD:
                msg.linear.x = 0.2;
                msg.angular.z = 0.0;
                pub.publish(msg);
                if (elapsedTime.toSec() >= 5.0) {
                    startTime = ros::Time::now();
                    state = MOVE_BACKWARD;
                }
                break;

            case MOVE_BACKWARD:
                msg.linear.x = -0.2;
                msg.angular.z = 0.0;
                pub.publish(msg);
                if (elapsedTime.toSec() >= 5.0) {
                    startTime = ros::Time::now();
                    state = ROTATE_CLOCKWISE;
                }
                break;

            case ROTATE_CLOCKWISE:
                msg.linear.x = 0.0;
                msg.angular.z = 0.3;
                pub.publish(msg);
                if (elapsedTime.toSec() >= 6.0) {
                    startTime = ros::Time::now();
                    state = ROTATE_COUNTER_CLOCKWISE;
                }
                break;

            case ROTATE_COUNTER_CLOCKWISE:
                msg.linear.x = 0.0;
                msg.angular.z = -0.3;
                pub.publish(msg);
                if (elapsedTime.toSec() >= 6.0) {
                    state = STOP;
                    shutdownInitiated = true; 
                }
                break;

            case STOP:
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                pub.publish(msg);
                if (shutdownInitiated) {
                    ros::Duration(1.0).sleep();
                    ROS_INFO_STREAM("----------[END WHEEL CONTROL TEST]-----------");
                    ros::shutdown();
                }
                break;
        }

        rate.sleep();
    }
}

std::string extractTopic(std::string key) {
    /*
     * Extracts topic names from configuration files based on platform (simulator/robot)
     * Reads configuration file to determine platform, then reads appropriate topic file
     *
     * @param:
     *     key: String key to search for in the topic configuration file
     *
     * @return:
     *     std::string: Topic name corresponding to the provided key
     *
     * @throws:
     *     Exits program if configuration files cannot be opened or key not found
     */
    
    bool debug = false;
    
    std::string configFileName = "actuatorTestConfiguration.ini";
    std::string packagePath;
    std::string configPathFile;
    
    std::string platformKey = "platform";
    std::string robotTopicKey = "robottopics";
    std::string simulatorTopicKey = "simulatortopics";
    std::string verboseModeKey = "verboseMode";

    std::string platformValue;
    std::string robotTopicValue;
    std::string simulatorTopicValue;
    
    std::string topicFileName;
    std::string topicPathFile;
    std::string topic_value = "";

    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
        promptAndExit(1);
    #endif

    configPathFile = packagePath + "/config/" + configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()) {
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;
    while(std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);

        if (paramKey == platformKey) { platformValue = paramValue; }
        else if (paramKey == robotTopicKey) { robotTopicValue = paramValue; }
        else if (paramKey == simulatorTopicKey) { simulatorTopicValue = paramValue; }
        else if (paramKey == verboseModeKey) { 
            transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);
            if (paramValue == "true") verboseMode = true;
            else verboseMode = false;
        }
    }
    configFile.close();

    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }    
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topicFileName.c_str());

    topicPathFile = packagePath + "/data/" + topicFileName;

    if (debug) printf("Topic file is %s\n", topicPathFile.c_str());

    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()) {
        printf("Unable to open the topic file %s\n", topicPathFile.c_str());
        promptAndExit(1);
    }

    std::string topicLineRead;
    while(std::getline(topicFile, topicLineRead)) {
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

std::string extractMode() {
    /*
     * Extracts the operational mode from configuration file
     * Determines whether the system should run in a specific test mode
     *
     * @param:
     *     None
     *
     * @return:
     *     std::string: Mode value from configuration file (converted to lowercase)
     *
     * @throws:
     *     Exits program if configuration file cannot be opened or mode not found
     */
    
    bool debug = false;
    
    std::string configFileName = "actuatorTestConfiguration.ini";
    std::string packagePath;
    std::string configPathFile;
    std::string modeKey = "mode";
    std::string modeValue;
    
    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
    #endif

    configPathFile = packagePath + "/config/" + configFileName;

    if (debug) printf("Config file is %s\n", configPathFile.c_str());

    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()) {
        printf("Unable to open the config file %s\n", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;
    
    while(std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == modeKey) { modeValue = paramValue; }
    }
    configFile.close();

    if (modeValue == "") {
        printf("Unable to find a valid mode.\n");
        promptAndExit(1);
    }
    return modeValue;
}

std::vector<std::string> extractTests() {
    /*
     * Extracts list of enabled tests from input configuration file
     * Reads test configuration and returns names of tests marked as "true"
     *
     * @param:
     *     test: Test category identifier (currently unused in implementation)
     *
     * @return:
     *     std::vector<std::string>: Vector of test names that are enabled (set to "true")
     *
     * @throws:
     *     Exits program if input configuration file cannot be opened
     */
    
    bool debug = false;
    
    std::string inputFileName = "actuatorTestInput.dat";
    std::string packagePath;
    std::string inputPathFile;
    
    std::vector<std::string> testName;

    #ifdef ROS
        packagePath = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        printf("ROS_PACKAGE_NAME is not defined. Please define the ROS_PACKAGE_NAME environment variable.\n");
        promptAndExit(1);
    #endif
    
    inputPathFile = packagePath + "/data/" + inputFileName;

    if (debug) printf("Input file is %s\n", inputPathFile.c_str());

    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()) {
        printf("Unable to open the input file %s\n", inputPathFile.c_str());
        promptAndExit(1);
    }

    std::string inpLineRead;
    std::string paramKey, paramValue;
    
    while(std::getline(inputFile, inpLineRead)) {
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue);
        transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramValue == "true") { 
            testName.push_back(paramKey);
        }
    }
    inputFile.close();

    std::cout<<"Tests to be executed: ";
    for (const auto& name : testName) {
        std::cout << name << " ";
    }
    std::cout << std::endl;

    return testName;
}

void promptAndExit(int status) {
    /*
     * Prompts user for input before exiting the program
     * Used for debugging and ensuring user sees error messages before program termination
     *
     * @param:
     *     status: Exit status code to return to the operating system
     *
     * @return:
     *     None (terminates the program)
     */
    
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void promptAndContinue() {
    /*
     * Prompts user for input before continuing program execution
     * Used for pausing execution to allow user to read output or prepare for next step
     *
     * @param:
     *     None
     *
     * @return:
     *     None
     */
    
    printf("Press any key to proceed ...\n");
    getchar();
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
     * Executes specified actuator tests in sequential order
     * Runs each test one after another, ensuring complete execution before starting the next
     *
     * @param:
     *     testNames: Vector of test names to execute (head, rarm, rhand, larm, lhand, leg, wheels)
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     *
     * @note:
     *     Exits program with error message if unknown test name is provided
     */
    
    for (const auto& testName : testNames) {
        if (testName == "head") {
            head(nh);
        } else if (testName == "rarm") {
            rArm(nh);
        } else if (testName == "rhand") {
            rHand(nh);
        } else if (testName == "larm") {
            lArm(nh);
        } else if (testName == "lhand") {
            lHand(nh);
        } else if (testName == "leg") {
            leg(nh);
        } else if (testName == "wheels") {
            wheels(nh);
        } else {
            std::cerr << "Unknown test provided: " << testName << ". Exiting...\n";
        }
    }
}

void executeTestsInParallel(const std::vector<std::string>& testNames, ros::NodeHandle& nh) {
    /*
     * Executes specified actuator tests in parallel using multiple threads
     * Creates separate threads for each test to run simultaneously, improving test efficiency
     * Note: Wheels test is excluded from parallel execution due to potential conflicts
     *
     * @param:
     *     testNames: Vector of test names to execute (head, rarm, rhand, larm, lhand, leg)
     *     nh: ROS NodeHandle for communication with ROS system
     *
     * @return:
     *     None
     *
     * @warning:
     *     Wheels test should not be run in parallel with other tests due to safety concerns
     *     and potential hardware conflicts
     */
    
    std::vector<std::thread> threads;
    for (const auto& testName : testNames) {
        if (testName == "head") {
            threads.push_back(std::thread(head, std::ref(nh)));
        } else if (testName == "rarm") {
            threads.push_back(std::thread(rArm, std::ref(nh)));
        } else if (testName == "rhand") {
            threads.push_back(std::thread(rHand, std::ref(nh)));
        } else if (testName == "larm") {
            threads.push_back(std::thread(lArm, std::ref(nh)));
        } else if (testName == "lhand") {
            threads.push_back(std::thread(lHand, std::ref(nh)));
        } else if (testName == "leg") {
            threads.push_back(std::thread(leg, std::ref(nh)));
        } else {
            std::cerr << "Unknown test provided: " << testName << ". Exiting...\n";
        }
    }

    for (auto& thread : threads) {
        thread.join();
    }
}