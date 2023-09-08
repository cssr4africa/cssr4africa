/*****************************************************************************************************************************
 * @file actuatorTest.cpp
 * @brief Controls the head, the left arm, the right arm, the leg and the wheels of Pepper robot by publishing a corresponding
 *        joint trajectories (for head, left arm, right arm, and leg) and twist (for wheels) messages.
 *        For the head, left arm, right arm and leg, the joints maximum, minimum and mid-range position will be demonstrated.
 *        For the wheels, a fixed linear and angular velocity for a fixed amount of time will be published.
 * @author CSSR4Africa Team
 * @version 1.0
 * @date September 07, 2023
 *  
 *****************************************************************************************************************************/

#include "pepper_interface_tests/actuatorTest.h"


// Function to create a JointTrajectoryPoint with time as a percentage of the range to control the velocity of the pepper robot
trajectory_msgs::JointTrajectoryPoint createJointTrajectoryPoint(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations, double percentage) {
    // Calculate time_from_start as a percentage of the range from 2.0 to 12.0
    double min_time = 2.0;
    double max_time = 12.0;
    // check if the percentage is within the range
    if (percentage < 0 || percentage > 100){
        ROS_ERROR_STREAM("Percentage must be between 0 and 100");
        return trajectory_msgs::JointTrajectoryPoint();
    }
    double time_from_start = min_time + ((100 - percentage) / 100.0) * (max_time - min_time);

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = accelerations;
    point.time_from_start = ros::Duration(time_from_start);
    return point;
}

// Function to publish a trajectory and adjust sleep time
void publishTrajectory(ros::Publisher& pub, trajectory_msgs::JointTrajectory& t_msg, double percentage) {
    double min_time = 2.0;
    double max_time = 12.0;
    if (percentage < 0 || percentage > 100){
        ROS_ERROR_STREAM("Percentage must be between 0 and 100");
        return;
    }
    double time_from_start = ((100- percentage) / 100.0) * (max_time - min_time);

    pub.publish(t_msg);
    double sleep_time = 6.0 + time_from_start; // Adjust sleep time based on percentage
    ros::Duration(sleep_time).sleep(); // Sleep for adjusted time
}

/* Main function */
int main(int argc, char **argv){
    // read the tests to execute from the config file
    std::vector<std::string> test_names;
    test_names = extract_tests("actuator");
    
    // initialize ros node 
    ros::init(argc, argv, "actuator_test");
    ros::NodeHandle nh;

    // run the tests 
    for (auto test : test_names){
        if (test == "Head"){
            headControl(nh);
            prompt_and_continue();
        }
        else if (test == "RArm"){
            rightArmControl(nh);  
            prompt_and_continue();
        }
        else if (test == "LArm"){
            leftArmControl(nh);
            prompt_and_continue();
        }
        else if (test == "Leg"){
            legControl(nh);
            prompt_and_continue();
        }
        else if (test == "Wheels"){
            wheelsControl(nh);
            prompt_and_continue();
        }
        else{
            std::cout << "No test provided. Exiting...\n";
            prompt_and_exit(1);
        }
    }
    return 0;
}

/* Test functions */

void headControl(ros::NodeHandle nh){
    // find the respective topic
    string topic_name = extract_topic("Head");
   
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1000, true);
    ros::Rate rate(50); // Loop at 50Hz until the node is shutdown

    // Joint names
    std::vector<std::string> name_vector = {"HeadPitch", "HeadYaw"};
    
    // Map joint names to indices
    std::map<std::string, int> name_index;
    for (int i = 0; i < name_vector.size(); ++i) {
        name_index[name_vector[i]] = i;
    }

    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {0.4451,   2.0857};
    std::vector<double> min_position = {-0.7068, -2.0857};

    std::vector<double> position_vector(2, 0.0); // Initialize with 2 elements, both set to 0.0
    std::vector<double> velocity_vector(2, 0.0);
    std::vector<double> acceleration_vector(2, 0.0);

    // Create the JointTrajectory message
    trajectory_msgs::JointTrajectory t_msg;
    t_msg.joint_names = name_vector;

    ROS_INFO_STREAM("----------[START HEAD CONTROL TEST]-----------");

    for (const std::string& joint : name_vector) {
        ROS_INFO_STREAM("[START] " << joint << " test.");
        
        ROS_INFO_STREAM("[MIN] Publishing a minimum position value");
        position_vector[name_index[joint]] = min_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 100));
        publishTrajectory(pub, t_msg, 100);

        ROS_INFO_STREAM("[MAX] Publishing a maximum position value");
        position_vector[name_index[joint]] = max_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 100));
        publishTrajectory(pub, t_msg, 80);

        ROS_INFO_STREAM("[MID] Publishing a mid-range position value.");
        position_vector[name_index[joint]] = (max_position[name_index[joint]] + min_position[name_index[joint]]) / 2;
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 100));
        publishTrajectory(pub, t_msg, 100);

        ROS_INFO_STREAM("[END] " << joint << " test.");
    }
    
    // End of the program
    ROS_INFO_STREAM("[SUCCESS] Head control test completed.");
    ROS_INFO_STREAM("                                      ");
}

void rightArmControl(ros::NodeHandle nh) {
    // Find the respective topic
    std::string topic_name = extract_topic("RArm");

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1000, true);
    ros::Rate rate(50); // Loop at 50Hz until the node is shutdown

    // Joint names for the right arm
    std::vector<std::string> name_vector = {"RShoulderPitch", "RShoulderRoll",  "RElbowRoll", "RElbowYaw", "RWristYaw"};

    // Map joint names to indices
    std::map<std::string, int> name_index;
    for (int i = 0; i < name_vector.size(); ++i) {
        name_index[name_vector[i]] = i;
    }

    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {2.0857,  -0.0087,  1.5620,  2.0857,  1.8239};
    std::vector<double> min_position = {-2.0857, -1.5620 , 0.0087, -2.0857, -1.5620};

    // Initialize position vector with zeros
    std::vector<double> position_vector(5, 0.0);
    std::vector<double> velocity_vector(5, 0.0);
    std::vector<double> acceleration_vector(5, 0.0);

    // Create the JointTrajectory message
    trajectory_msgs::JointTrajectory t_msg;
    t_msg.joint_names = name_vector;

    ROS_INFO_STREAM("----------[START RIGHT ARM CONTROL TEST]-------------");

    for (const std::string& joint : name_vector) {
        ROS_INFO_STREAM("[START] " << joint << " test.");
        
        ROS_INFO_STREAM("[MIN] Publishing a minimum position value");
        position_vector[name_index[joint]] = min_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 75);

        ROS_INFO_STREAM("[MAX] Publishing a maximum position value");
        position_vector[name_index[joint]] = max_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 50);

        ROS_INFO_STREAM("[MID] Publishing a mid-range position value.");
        position_vector[name_index[joint]] = (max_position[name_index[joint]] + min_position[name_index[joint]]) / 2;
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 75);

        ROS_INFO_STREAM("[END] " << joint << " test.");
    }

    // Put down right arm
    ROS_INFO_STREAM("[PUT DOWN RIGHT ARM] Publishing default position values.");
    position_vector = {1.45, -0.1, 0.1, 0.0, 1.0};
    t_msg.points.clear(); // Clear previous points
    t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 0.0)); // Set a specific time
    publishTrajectory(pub, t_msg, 0.0);

    // End of the program
    ROS_INFO_STREAM("[SUCCESS] Right arm control test completed.");
    ROS_INFO_STREAM("                                           ");
}


void leftArmControl(ros::NodeHandle nh){
    // find the respective topic
    string topic_name = extract_topic("LArm");

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1000, true);
    ros::Rate rate(50); // Loop at 50Hz until the node is shutdown

    // Joint names for the left arm
    std::vector<std::string> name_vector = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};

    // Map joints names to indices
    std::map<std::string, int> name_index;
    for (int i = 0; i < name_vector.size(); ++i) {
        name_index[name_vector[i]] = i;
    }

    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {2.0857, 1.5620,  -0.0087,  2.0857,  1.8239};
    std::vector<double> min_position = {-2.0857, 0.0087, -1.5620, -2.0857, -1.8239};

    // Initialize position vector with zeros
    std::vector<double> position_vector(5, 0.0);
    std::vector<double> velocity_vector(5, 0.0);
    std::vector<double> acceleration_vector(5, 0.0);

     // Create the JointTrajectory message
    trajectory_msgs::JointTrajectory t_msg;
    t_msg.joint_names = name_vector;

    ROS_INFO_STREAM("----------[START LEFT ARM CONTROL TEST]----------");
 
    for (const std::string& joint : name_vector) {
        ROS_INFO_STREAM("[START] " << joint << " test.");

        ROS_INFO_STREAM("[MAX] Publishing a maximum position value");
        position_vector[name_index[joint]] = max_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 75);

        ROS_INFO_STREAM("[MIN] Publishing a minimum position value");
        position_vector[name_index[joint]] = min_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 50);

        ROS_INFO_STREAM("[MID] Publishing a mid-range position value.");
        position_vector[name_index[joint]] = (max_position[name_index[joint]] + min_position[name_index[joint]]) / 2;
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 75);

        ROS_INFO_STREAM("[END] " << joint << " test.");
    }
    
    ROS_INFO_STREAM("[PUT DOWN LEFT ARM] Publishing [1.45, 0.1, -0.1, 0.0, -1.0] position values.");
    position_vector = {1.45, 0.1, -0.1, 0.0, -1.0};
    t_msg.points.clear(); // Clear previous points
    t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 0.0)); // Set a specific time
    publishTrajectory(pub, t_msg, 0.0);

    // End of the program
    ROS_INFO_STREAM("[SUCCESS] Right arm control test completed.");
    ROS_INFO_STREAM("                                           ");

}

void legControl(ros::NodeHandle nh){
    // find the respective topic
    string topic_name = extract_topic("Leg");

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1000, true);
    ros::Rate rate(50); // Loop at 50Hz until the node is shutdown

    // Joint names for the leg
    std::vector<std::string> name_vector = {"HipPitch", "HipRoll", "KneePitch"};

    // Map joint names to indices
    std::map<std::string, int> name_index;
    for (int i = 0; i < name_vector.size(); ++i) {
        name_index[name_vector[i]] = i;
    }

    // Maximum and minimum positions for each joint
    std::vector<double> max_position = {1.0385,   0.5149,   0.5149};
    std::vector<double> min_position = {-1.0385, -0.5149 , -0.5149};

    std::vector<double> position_vector(3, 0.0); // Initialize with 5 elements, all set to 0.0
    std::vector<double> velocity_vector(3, 0.0);
    std::vector<double> acceleration_vector(3, 0.0);
    
    // Create the JointTrajectory message
    trajectory_msgs::JointTrajectory t_msg;
    t_msg.joint_names = name_vector;

    ROS_INFO_STREAM("------[START LEG JOINT CONTROL TEST]-------");

    for (const std::string& joint : name_vector) {
        ROS_INFO_STREAM("[START] " << joint << " test.");

        ROS_INFO_STREAM("[MAX] Publishing a maximum position value");
        position_vector[name_index[joint]] = max_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 50);

        ROS_INFO_STREAM("[MIN] Publishing a minimum position value");
        position_vector[name_index[joint]] = min_position[name_index[joint]];
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 25);

        ROS_INFO_STREAM("[MID] Publishing a mid-range position value.");
        position_vector[name_index[joint]] = (max_position[name_index[joint]] + min_position[name_index[joint]]) / 2;
        t_msg.points.clear(); // Clear previous points
        t_msg.points.push_back(createJointTrajectoryPoint(position_vector, velocity_vector, acceleration_vector, 75));
        publishTrajectory(pub, t_msg, 25);

        ROS_INFO_STREAM("[END] " << joint << " test.");
    }    
    // End of the program
    ROS_INFO_STREAM("[SUCCESS] Leg joint control test completed.");
    ROS_INFO_STREAM("                                           ");
}

// Function to publish a velocity command to a joint
void publish_velocity(ros::Publisher &pub, geometry_msgs::Twist &msg, ros::Rate &rate, double duration) {
    ros::Time startTime = ros::Time::now();
    ros::Duration waitTime = ros::Duration(duration); 
    ros::Time endTime = startTime + waitTime;
    // Publish the trajectory for 1 seconds
    while(ros::ok() && ros::Time::now() < endTime) {
        pub.publish(msg);
        rate.sleep();
    }
}

void wheelsControl(ros::NodeHandle nh){
    // find the respective topic
   string topic_name = extract_topic("Wheels");
   
   // Create a publisher to publish geometry_msgs::Twist messages on the /pepper/cmd_vel topic
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1000, true);

   // Set the publishing rate to 50 Hz
   ros::Rate rate(50); 

   int i = 1; // create iterator variable
   // Create a Twist message object
   geometry_msgs::Twist msg;
   
   ROS_INFO_STREAM("-------[START WHEEL CONTROL TEST]--------");
   /* [1] THIS SECTION PUBLISHES A LINEAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[LINEAR VELOCITY START] Publishing linear velocity on the cmd vel started.");
   
   // Initialize the message with 0 linear velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity
   publish_velocity(pub, msg, rate, 1);

   // Publish a fixed positive linear velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.linear.x = 0.05;

   // Publish the positive velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 2);

   // Publish a fixed negative linear velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.linear.x = -0.05;

   // Publish the negative velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset linear velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.linear.x = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[LINEAR VELOCITY END] Publishing linear velocity ended.");
   
   /* [2] THIS SECTION PUBLISHES AN ANGULAR VELOCITY ON THE CMD VEL TOPIC */
   ROS_INFO_STREAM("[ANGULAR VELOCITY START] Publishing angular velocity on the cmd vel started.");
   
   // Initialize the message with 0 angular velocity
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 2);

   // Publish a fixed positive angular velocity
   ROS_INFO_STREAM("[POSITIVE VELOCITY] Publishing a fixed positive velocity value");
   msg.angular.z = 0.3925;

   // Publish the positive velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 1);

   // Publish a fixed negative angular velocity
   ROS_INFO_STREAM("[NEGATIVE VELOCITY] Publishing a fixed negative velocity value");
   msg.angular.z = -0.3925;

   // Publish the negative velocity 
   publish_velocity(pub, msg, rate, 4);

   // Reset angular velocity to 0
   ROS_INFO_STREAM("[ZERO VELOCITY] Publishing 0 velocity value.");
   msg.angular.z = 0.0;

   // Publish 0 velocity 
   publish_velocity(pub, msg, rate, 4);
   
   ROS_INFO_STREAM("[ANGULAR VELOCITY END] Publishing angular velocity ended.");
    
   // Print success message
   ROS_INFO_STREAM("[SUCCESS] Wheel control test completed.");
   ROS_INFO_STREAM("                                       ");
}


/* Helper Functions */
void prompt_and_exit(int status){
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void prompt_and_continue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

/* Extract topic names for the respective simulator or physical robot */
string extract_topic(string key){
    bool debug = false;   // used to turn debug message on
    
    std::string conf_file = "actuatorTestConfiguration.ini";  // configuration filename
    std::string config_path;                                  // configuration path
    std::string config_path_and_file;                         // configuration path and filename
    
    std::string platformKey = "platform";                     // platform key 
    std::string robotTopicKey = "robotTopics";                // robot topic key
    std::string simulatorTopicKey = "simulatorTopics";        // simulator topic key

    std::string platformValue;                                // platform value
    std::string robotTopicValue;                              // robot topic value
    std::string simulatorTopicValue;                          // simulator topic value
    
    std::string topic_file;                                   // topic filename
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += conf_file;

    if (debug) printf("Config file is %s\n", config_path_and_file.c_str());

    // Open configuration file
    std::ifstream conf_if(config_path_and_file.c_str());
    if (!conf_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(conf_if, configLineRead)){
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
    conf_if.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topic_file = simulatorTopicValue; }
    else if (platformValue == "robot") { topic_file = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topic_file.c_str());

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file;

    if (debug) printf("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        printf("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topicLineRead)){
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
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        prompt_and_exit(1);
    }
    return topic_value;
}


/* Extract the expected tests to run for the respective actuator or sensor tests */
std::vector<std::string> extract_tests(std::string set){
    bool debug = false;   // used to turn debug message on
    
    std::string inp_file;                                  // input filename
    std::string inp_path;                                  // input path
    std::string inp_path_and_file;                         // input path and filename
    
    std::vector<std::string> test_name;
    std::string flag;

    if (set == "actuator"){
        inp_file = "actuatorTestInput.dat";
    }
    else{
        inp_file = "sensorTestInput.dat";
    }

    // Construct the full path of the input file
    #ifdef ROS
        inp_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        inp_path = "..";
    #endif
    
    inp_path += "/data/";
    inp_path_and_file = inp_path;
    inp_path_and_file += inp_file;

    if (debug) printf("Input file is %s\n", inp_path_and_file.c_str());

    // Open input file
    std::ifstream inp_if(inp_path_and_file.c_str());
    if (!inp_if.is_open()){
        printf("Unable to open the input file %s\n", inp_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string inpLineRead;  // variable to read the line in the file
    
    std::string paramKey, paramValue;
    // Get key-value pairs from the input file
    while(std::getline(inp_if, inpLineRead)){
        std::istringstream iss(inpLineRead);
    
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        
        trim(paramValue); // trim whitespace
        transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower); // convert to lower case

        if (paramValue == "true"){ test_name.push_back(paramKey);}
    }
    inp_if.close();

    return test_name;
}

