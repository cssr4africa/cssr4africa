/* behaviorControllerTestDriver.cpp   Source code for the simulated topics (drivers)
 *
 * Author: Tsegazeab Taye Tefferi
 * Date: April 25, 2025
 * Version: 1.0
 *
 * Copyright (C) 2023 CSSR4Africa Consortium
 *
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 *
 * Website: www.cssr4africa.org
 *
 * This program comes with ABSOLUTELY NO WARRANTY.
 *
 */

#include <behaviorControllerTest/behaviorControllerTestInterface.h>
#include <math.h>
#include <time.h>

#include <ctime>

#include "std_msgs/String.h"
#include "unit_tests/overtAttentionMode.h"

int arrivalRate = 1000;
std::string traversal = "complete";
bool verboseMode;

static int samplePoisson(double lambda)
{
    /* Generate a random sample from a Poisson distribution with a given mean, lambda */
    /* Use the function rand to generate a random number */
    static bool first_call = true;
    int count;
    double product;
    double zero_probability;
    /* Seed the random-number generator with current time so */
    /* that the numbers will be different every time we run */
    if (first_call) {
        srand((unsigned)time(NULL));
        first_call = false;
    }
    count = 0;
    product = (double)rand() / (double)RAND_MAX;
    zero_probability = exp(-lambda);
    while (product > zero_probability) {
        count++;
        product = product * ((double)rand() / (double)RAND_MAX);
    }
    return (count);
}

static int getCount(int ari = 1, long int inc = 10000)
{
    /* example usage of samplePoisson */
    int arrival_rate_input = ari;  // the arrival rate in events per minute
    double arrival_rate;           // the arrival rate in events per millisecond
    long int increment = inc;      // the period of each simulation interval in milliseconds
    int count;                     // the number of events that arrive in any given
    // simulation interval (i.e time increment)
    double lambda;  // the mean number of events that arrive in any one
    // similation interval (i.e. time increment)
    /* arrival_rate_input is in events per second so convert to events per millisecond */
    arrival_rate = ((float)arrival_rate_input) / (60 * 1000);
    /* the Poisson distribution mean, lambda, is the arrival rate of events during */
    /* the simulation interval, i.e. arrival rate per millisecond multiplied by */
    /* the number of milliseconds in each simulation interval */
    lambda = arrival_rate * increment;
    /* Compute the number of events that have arrived in the current simulation interval */
    count = samplePoisson(lambda);
    return count;
}

void speechEventTextTopic(ros::Publisher &publisher)
{
    std::string detectedText;
    std::vector<std::string> possibleAffirmativeResponses = {
        "yes",
        "i would love that",
        "absolutely",
        "go ahead",
        "i'd be happy to",
        "sounds good",
        "sounds great"};
    int responseIndex = 0;
    std_msgs::String msg;
    int count = getCount(arrivalRate);

    if (traversal == "partial02") {  // Visitor is Silent
        return;
    }

    if (traversal == "partial03") {  // Visitor responds with a 'No'
        msg.data = "no";
        publisher.publish(msg);
        return;
    }

    if (count > 0) {
        if (hasSucceeded()) {
            responseIndex = static_cast<int>((rand() % possibleAffirmativeResponses.size()));
            detectedText = possibleAffirmativeResponses[responseIndex];
        } else {
            detectedText = "no";
        }
        msg.data = detectedText;
        publisher.publish(msg);
    }
}

void overtAttentionModeTopic(ros::Publisher &publisher)
{
    std::string mode;
    static int modeValue = 1;
    static bool timerStarted = false;
    static time_t start;

    unit_tests::overtAttentionMode msg;
    if (ros::param::get("/overtAttentionMode", mode) && ros::param::get("/overtAttentionModeStatus", modeValue)) {
        msg.state = mode;
        msg.value = modeValue;

    if (((msg.state == "seeking") || (msg.state == "scanning")) || ((msg.state == "social") && msg.value == 1)) {
            if ((traversal == "partial01" && msg.state == "seeking")||(traversal == "partial04" && msg.state == "social"))  // Mutual gaze fails or Visitor leaves tour midway
            {
                modeValue = 3;  // FAILED
                printMsg(WARNING_MSG,"[/overtAttention/mode]: <" + msg.state + "> failed");
                timerStarted = false;
            }

            if (!timerStarted) {
                start = time(NULL);
            }
            if (time(NULL) - start < 10 && modeValue == 1) {
                int count = getCount(arrivalRate);
                if (count > 0) {
                    modeValue = 2;  // SUCCESS
                    printMsg(INFO_MSG,"[/overtAttention/mode]: <"+ msg.state +"> succeeded");
                    timerStarted = false;
                }
            }
            if (time(NULL) - start >= 10 && msg.value == 1) {
                modeValue = 3;  // FAILED
                printMsg(WARNING_MSG,"[/overtAttention/mode]: <" + msg.state + "> failed");
                timerStarted = false;
            }
        }
    } else {
        msg.state = "disabled";
        msg.value = 1;
    }
    ros::param::set("/overtAttentionModeStatus", modeValue);
    publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviorControllerTestDriver");
    ros::NodeHandle nh;

    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try {
        arrivalRate = std::stof(getValueFromConfig("arrivalRate"));
        failureRate = std::stof(getValueFromConfig("failureRate"));
        traversal = getValueFromConfig("traversal");
        verboseMode = (getValueFromConfig("verboseMode") == "true");
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Fatal Error: " << e.what());
        ros::shutdown();
        return 0;
    }

    printMsg(INFO_MSG,"Traversal: " + traversal);

    printMsg(INFO_MSG,"Incidence arrival rate set at: " + std::to_string(arrivalRate));

    ros::Publisher speechEventpublisher = nh.advertise<std_msgs::String>("/speechEvent/text", 10);
    printMsg(INFO_MSG,"/speechEvent/text Ready");

    ros::Publisher overtAttentionPublisher = nh.advertise<unit_tests::overtAttentionMode>("/overtAttention/mode", 10);
    printMsg(INFO_MSG,"/overtAttention/mode Ready");
    ros::Rate rate(1);

    srand(time(0));

    while (ros::ok()) {
        speechEventTextTopic(speechEventpublisher);
        overtAttentionModeTopic(overtAttentionPublisher);
        rate.sleep();
    }
}