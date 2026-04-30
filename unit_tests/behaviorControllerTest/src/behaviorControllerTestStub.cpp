/* behaviorControllerTestStub.cpp   Source code for the simulated services (stubs)
 *
 * Author: Tsegazeab Taye Tefferi
 * Date: April 25, 2025
 * Version: 1.0
 *
 * Author: Tsegazeab Taye Tefferi, Carnegie Mellon University Africa
 * Email: ttefferi@andrew.cmu.edu
 * Date: April 20, 2026
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
 *
 */

#include <behaviorControllerTest/behaviorControllerTestInterface.h>

#include "unit_tests/animateBehaviorSetActivation.h"
#include "unit_tests/gestureExecutionPerformGesture.h"
#include "unit_tests/gestureExecutionPerformGestureAction.h"
#include "unit_tests/overtAttentionSetMode.h"
#include "unit_tests/robotLocalizationSetPose.h"
#include "unit_tests/robotNavigationSetGoal.h"
#include "unit_tests/robotNavigationSetGoalAction.h"
#include "unit_tests/robotNavigationSetPoseAction.h"
#include "unit_tests/speechEventSetLanguage.h"
#include "unit_tests/speechEventSetStatus.h"
#include "unit_tests/tabletEventPromptAndGetResponse.h"
#include "unit_tests/textToSpeechSayText.h"
#include "unit_tests/textToSpeechSayTextAction.h"

bool verboseMode = true;

bool animateBehaviorSetActivationHandler(unit_tests::animateBehaviorSetActivation::Request& req, unit_tests::animateBehaviorSetActivation::Response& res)
{
    printMsg(INFO_MSG, "[/animateBehavior/setActivation]: Set state: " + req.state);

    // do success or failure
    std::string success = hasSucceeded() ? "1" : "0";
    res.success = success;
    return true;
}

bool gestureExecutionPerformGestureHandler(unit_tests::gestureExecutionPerformGesture::Request& req, unit_tests::gestureExecutionPerformGesture::Response& res)
{
    std::stringstream msg;
    msg << "[/gestureExecution/perform_gesture]: Performing Gesture:"
           "\n\t type: "
        << req.gesture_type << "\n\t id: " << req.gesture_id << "\n\t arm: " << req.arm << "\n\t palm_orientation: " << req.deictic_shape << "\n\t duration: " << req.gesture_duration << "\n\t bow_nod_angle: " << req.bow_nod_angle << "\n\t location_x: " << req.location_x << "\n\t location_y: " << req.location_y << "\n\t location_z: " << req.location_z;

    printMsg(INFO_MSG, msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.gesture_success = success;

    return true;
}

bool gestureExecutionPerformGestureActionHandler(const actionlib::SimpleActionServer<unit_tests::gestureExecutionPerformGestureAction>::GoalConstPtr& goal, actionlib::SimpleActionServer<unit_tests::gestureExecutionPerformGestureAction>* as)
{
    unit_tests::gestureExecutionPerformGestureFeedback feedback;
    unit_tests::gestureExecutionPerformGestureResult result;

    std::stringstream msg;
    msg << "[/gestureExecution/perform_gesture (action)]: Performing Gesture:"
           "\n\t type: "
        << goal->gesture_type << "\n\t id: " << goal->gesture_id << "\n\t arm: " << goal->arm << "\n\t palm_orientation: " << goal->deictic_shape << "\n\t duration: " << goal->gesture_duration << "\n\t bow_nod_angle: " << goal->bow_nod_angle << "\n\t location_x: " << goal->location_x << "\n\t location_y: " << goal->location_y << "\n\t location_z: " << goal->location_z;

    printMsg(INFO_MSG, msg.str());

    // Check for preemption before doing the "work"
    if (as->isPreemptRequested() || !ros::ok()) {
        printMsg(INFO_MSG, "[/gestureExecution/perform_gesture (action)]: Preempted");
        as->setPreempted();
        return true;
    }

    as->publishFeedback(feedback);

    // Roll the dice
    bool success = hasSucceeded();
    result.gesture_success = success;

    if (success) {
        as->setSucceeded(result);
    } else {
        as->setAborted(result);
    }

    return true;
}

bool overtAttentionSetModeHandler(unit_tests::overtAttentionSetMode::Request& req, unit_tests::overtAttentionSetMode::Response& res)
{
    printMsg(INFO_MSG, "[/overtAttention/set_mode]: Set mode: " + req.state);

    // do success or failure
    bool success = hasSucceeded();
    ros::param::set("/overtAttentionModeStatus", 1);
    ros::param::set("/overtAttentionMode", req.state);

    res.mode_set_success = success;
    return true;
}

bool robotLocalizationSetPoseHandler(unit_tests::robotLocalizationSetPose::Request& req, unit_tests::robotLocalizationSetPose::Response& res)
{
    std::stringstream msg;
    msg << "[/robotNavigation/reset_pose]: Set pose: " << "\n\t x: " << req.x << "\n\t y: " << req.y << "\n\t theta: " << req.theta;
    printMsg(INFO_MSG, msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool robotNavigationSetGoalHandler(unit_tests::robotNavigationSetGoal::Request& req, unit_tests::robotNavigationSetGoal::Response& res)
{
    std::stringstream msg;
    msg << "[/robotNavigation/set_goal]: Set goal: " << "\n\t goal_x: " << req.goal_x << "\n\t goal_y: " << req.goal_y << "\n\t goal_theta: " << req.goal_theta;
    printMsg(INFO_MSG, msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.navigation_goal_success = success;
    return true;
}

void robotNavigationSetGoalActionHandler(const actionlib::SimpleActionServer<unit_tests::robotNavigationSetGoalAction>::GoalConstPtr& goal, actionlib::SimpleActionServer<unit_tests::robotNavigationSetGoalAction>* as)
{
    unit_tests::robotNavigationSetGoalFeedback feedback;
    unit_tests::robotNavigationSetGoalResult result;

    std::stringstream msg;
    msg << "[/robotNavigation/set_goal (action)]: Set goal: "
        << "\n\t goal_x: " << goal->goal_x
        << "\n\t goal_y: " << goal->goal_y
        << "\n\t goal_theta: " << goal->goal_theta;
    printMsg(INFO_MSG, msg.str());

    if (as->isPreemptRequested() || !ros::ok()) {
        printMsg(INFO_MSG, "[/robotNavigation/set_goal (action)]: Preempted");
        as->setPreempted();
        return;
    }

    bool success = hasSucceeded();
    result.navigation_goal_success = success;

    if (success)
        as->setSucceeded(result);
    else
        as->setAborted(result);
}

bool speechEventSetStatusHandler(unit_tests::speechEventSetStatus::Request& req, unit_tests::speechEventSetStatus::Response& res)
{
    printMsg(INFO_MSG, "[/speechEvent/set_enabled]: Set enabled: " + req.status);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool speechEventSetLanguageHandler(unit_tests::speechEventSetLanguage::Request& req, unit_tests::speechEventSetLanguage::Response& res)
{
    printMsg(INFO_MSG, "[/speechEvent/set_language]: Set language: " + req.language);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool tabletEventPromptAndGetResponseHandler(unit_tests::tabletEventPromptAndGetResponse::Request& req, unit_tests::tabletEventPromptAndGetResponse::Response& res)
{
    printMsg(INFO_MSG, "[/textToSpeech/say_text]: Prompt Message: '" + req.message);

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool textToSpeechSayTextHandler(unit_tests::textToSpeechSayText::Request& req, unit_tests::textToSpeechSayText::Response& res)
{
    printMsg(INFO_MSG, "[/textToSpeech/say_text]: Say text: '" + req.message + "' in '" + req.language + "'");

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

void textToSpeechSayTextActionHandler(const actionlib::SimpleActionServer<unit_tests::textToSpeechSayTextAction>::GoalConstPtr& goal, actionlib::SimpleActionServer<unit_tests::textToSpeechSayTextAction>* as)
{
    unit_tests::textToSpeechSayTextFeedback feedback;
    unit_tests::textToSpeechSayTextResult result;

    printMsg(INFO_MSG, "[/textToSpeech/say_text (action)]: Say text: '" + goal->text + "' in '" + goal->language + "'");

    if (as->isPreemptRequested() || !ros::ok()) {
        printMsg(INFO_MSG, "[/textToSpeech/say_text (action)]: Preempted");
        as->setPreempted();
        return;
    }

    bool success = hasSucceeded();
    result.success = success;

    if (success)
        as->setSucceeded(result);
    else
        as->setAborted(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behaviorControllerTestStub");
    ros::NodeHandle nh;

    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try {
        failureRate = std::stof(getValueFromConfig("failureRate"));
        verboseMode = (getValueFromConfig("verboseMode") == "true");
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal Error: " << e.what());
        ros::shutdown();
        return 0;
    }

    printMsg(INFO_MSG, "Server failure Rate set at: " + std::to_string(failureRate));

    /* Advertising the services */
    ros::ServiceServer animateBehaviorSetActivationServer = nh.advertiseService("/animateBehaviour/setActivation", animateBehaviorSetActivationHandler);
    printMsg(INFO_MSG, animateBehaviorSetActivationServer.getService() + " Ready");

    ros::ServiceServer gestureExecutionPerformGestureServer = nh.advertiseService("/gestureExecution/perform_gesture", gestureExecutionPerformGestureHandler);
    printMsg(INFO_MSG, gestureExecutionPerformGestureServer.getService() + " Ready");

    ros::ServiceServer overtAttentionSetModeServer = nh.advertiseService("/overtAttention/set_mode", overtAttentionSetModeHandler);
    printMsg(INFO_MSG, overtAttentionSetModeServer.getService() + " Ready");

    ros::ServiceServer robotLocalizationSetPoseServer = nh.advertiseService("/robotLocalization/set_pose", robotLocalizationSetPoseHandler);
    printMsg(INFO_MSG, robotLocalizationSetPoseServer.getService());

    ros::ServiceServer robotNavigationSetGoalServer = nh.advertiseService("/robotNavigation/set_goal", robotNavigationSetGoalHandler);
    printMsg(INFO_MSG, robotNavigationSetGoalServer.getService() + " Ready");

    ros::ServiceServer speechEventSetStatusServer = nh.advertiseService("/speechEvent/set_language", speechEventSetLanguageHandler);
    printMsg(INFO_MSG, speechEventSetStatusServer.getService() + " Ready");

    ros::ServiceServer speechEventSetLanguageServer = nh.advertiseService("/speechEvent/set_enabled", speechEventSetStatusHandler);
    printMsg(INFO_MSG, speechEventSetLanguageServer.getService() + " Ready");

    ros::ServiceServer tabletEventPromptAndGetResponseServer = nh.advertiseService("/textToSpeech/say_text", textToSpeechSayTextHandler);
    printMsg(INFO_MSG, tabletEventPromptAndGetResponseServer.getService() + " Ready");

    ros::ServiceServer textToSpeechSayTextServer = nh.advertiseService("/tabletEvent/prompt_and_get_response", tabletEventPromptAndGetResponseHandler);
    printMsg(INFO_MSG, textToSpeechSayTextServer.getService() + " Ready");

    // Create the action server
    actionlib::SimpleActionServer<unit_tests::gestureExecutionPerformGestureAction> gs(nh, "/gestureExecution/perform_gesture", boost::bind(&gestureExecutionPerformGestureActionHandler, _1, &gs), false);
    gs.start();
    ROS_INFO("gestureExecution Action server started. Waiting for goals...");

    actionlib::SimpleActionServer<unit_tests::textToSpeechSayTextAction> tts(nh, "/textToSpeech/say_text", boost::bind(&textToSpeechSayTextActionHandler, _1, &tts), false);
    tts.start();
    ROS_INFO("textToSpeech Action server started. Waiting for goals...");

    actionlib::SimpleActionServer<unit_tests::robotNavigationSetGoalAction> ngs(nh, "/robotNavigation/set_goal", boost::bind(&robotNavigationSetGoalActionHandler, _1, &ngs), false);
    ngs.start();
    ROS_INFO("robotNavigation/set_goal Action server started. Waiting for goals...");

    ros::spin();
    return 0;
}