/* behaviorControllerTestStub.cpp   Source code for the simulated services (stubs)
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

#include "unit_tests/animateBehaviorSetActivation.h"
#include "unit_tests/gestureExecutionPerformGesture.h"
#include "unit_tests/overtAttentionSetMode.h"
#include "unit_tests/robotLocalizationSetPose.h"
#include "unit_tests/robotNavigationSetGoal.h"
#include "unit_tests/speechEventSetStatus.h"
#include "unit_tests/speechEventSetLanguage.h"
#include "unit_tests/tabletEventPromptAndGetResponse.h"
#include "unit_tests/textToSpeechSayText.h"

bool verboseMode = true;

bool animateBehaviorSetActivationHandler(unit_tests::animateBehaviorSetActivation::Request &req, unit_tests::animateBehaviorSetActivation::Response &res)
{

    printMsg(INFO_MSG,"[/animateBehavior/setActivation]: Set state: " + req.state);

    // do success or failure
    std::string success = hasSucceeded() ?  "1": "0";
    res.success = success;
    return true;
}

bool gestureExecutionPerformGestureHandler(unit_tests::gestureExecutionPerformGesture::Request &req, unit_tests::gestureExecutionPerformGesture::Response &res)
{

    std::stringstream msg;
    msg<<"[/gestureExecution/perform_gesture]: Performing Gesture:"
                    "\n\t type: " << req.gesture_type << 
                    "\n\t id: " << req.gesture_id << 
                    "\n\t duration: " << req.gesture_duration << 
                    "\n\t bow_nod_angle: " << req.bow_nod_angle << 
                    "\n\t location_x: " << req.location_x << 
                    "\n\t location_y: " << req.location_y << 
                    "\n\t location_z: " << req.location_z;
    
    printMsg(INFO_MSG,msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.gesture_success = success;
    return true;
}

bool overtAttentionSetModeHandler(unit_tests::overtAttentionSetMode::Request &req, unit_tests::overtAttentionSetMode::Response &res)
{

    printMsg(INFO_MSG,"[/overtAttention/set_mode]: Set mode: " + req.state);

    // do success or failure
    bool success = hasSucceeded();
    ros::param::set("/overtAttentionModeStatus",1);
    ros::param::set("/overtAttentionMode",req.state);

    res.mode_set_success = success;
    return true;
}

bool robotLocalizationSetPoseHandler(unit_tests::robotLocalizationSetPose::Request &req, unit_tests::robotLocalizationSetPose::Response &res)
{

    std::stringstream msg;
    msg<< "[/robotNavigation/reset_pose]: Set pose: " << 
                    "\n\t x: " << req.x << 
                    "\n\t y: " << req.y << 
                    "\n\t theta: " << req.theta;
    printMsg(INFO_MSG, msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool robotNavigationSetGoalHandler(unit_tests::robotNavigationSetGoal::Request &req, unit_tests::robotNavigationSetGoal::Response &res)
{

    std::stringstream msg;
    msg<< "[/robotNavigation/set_goal]: Set goal: " << 
                    "\n\t goal_x: " << req.goal_x << 
                    "\n\t goal_y: " << req.goal_y << 
                    "\n\t goal_theta: " << req.goal_theta;
    printMsg(INFO_MSG, msg.str());

    // do success or failure
    bool success = hasSucceeded();
    res.navigation_goal_success = success;
    return true;
}

bool speechEventSetStatusHandler(unit_tests::speechEventSetStatus::Request &req, unit_tests::speechEventSetStatus::Response &res)
{

    printMsg(INFO_MSG,"[/speechEvent/set_enabled]: Set enabled: " + req.status);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool speechEventSetLanguageHandler(unit_tests::speechEventSetLanguage::Request &req, unit_tests::speechEventSetLanguage::Response &res)
{

    printMsg(INFO_MSG,"[/speechEvent/set_language]: Set language: " + req.language);

    // do success or failure
    bool success = hasSucceeded();
    res.response = success;
    return true;
}

bool tabletEventPromptAndGetResponseHandler(unit_tests::tabletEventPromptAndGetResponse::Request &req, unit_tests::tabletEventPromptAndGetResponse::Response &res)
{

    printMsg(INFO_MSG,"[/textToSpeech/say_text]: Prompt Message: '" + req.message);

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

bool textToSpeechSayTextHandler(unit_tests::textToSpeechSayText::Request &req, unit_tests::textToSpeechSayText::Response &res)
{

    printMsg(INFO_MSG,"[/textToSpeech/say_text]: Say text: '" + req.message + "' in '" + req.language + "'");

    // do success or failure
    bool success = hasSucceeded();
    res.success = success;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviorControllerTestStub");
    ros::NodeHandle nh;

    /* Retrieve the values from the configuration file       */
    /* Display the error and exit, if the file is unreadable */
    try
    {
        failureRate = std::stof(getValueFromConfig("failureRate"));
        verboseMode = (getValueFromConfig("verboseMode") == "true");
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Fatal Error: "<<e.what());
        ros::shutdown();
        return 0;
    }    

    printMsg(INFO_MSG,"Server failure Rate set at: " + std::to_string(failureRate));


    /* Advertising the services */
    ros::ServiceServer animateBehaviorSetActivationServer = nh.advertiseService("/animateBehaviour/setActivation", animateBehaviorSetActivationHandler);
    printMsg(INFO_MSG,animateBehaviorSetActivationServer.getService()+" Ready");
    
    ros::ServiceServer gestureExecutionPerformGestureServer = nh.advertiseService("/gestureExecution/perform_gesture", gestureExecutionPerformGestureHandler);
    printMsg(INFO_MSG,gestureExecutionPerformGestureServer.getService()+" Ready");

    ros::ServiceServer overtAttentionSetModeServer = nh.advertiseService("/overtAttention/set_mode", overtAttentionSetModeHandler);
    printMsg(INFO_MSG,overtAttentionSetModeServer.getService()+" Ready");

    ros::ServiceServer robotLocalizationSetPoseServer = nh.advertiseService("/robotLocalization/set_pose", robotLocalizationSetPoseHandler);
    printMsg(INFO_MSG,robotLocalizationSetPoseServer.getService());

    ros::ServiceServer robotNavigationSetGoalServer = nh.advertiseService("/robotNavigation/set_goal", robotNavigationSetGoalHandler);
    printMsg(INFO_MSG,robotNavigationSetGoalServer.getService()+" Ready");

    ros::ServiceServer speechEventSetStatusServer = nh.advertiseService("/speechEvent/set_language", speechEventSetLanguageHandler);
    printMsg(INFO_MSG,speechEventSetStatusServer.getService()+" Ready");

    ros::ServiceServer speechEventSetLanguageServer = nh.advertiseService("/speechEvent/set_enabled", speechEventSetStatusHandler);
    printMsg(INFO_MSG,speechEventSetLanguageServer.getService()+" Ready");

    ros::ServiceServer tabletEventPromptAndGetResponseServer = nh.advertiseService("/textToSpeech/say_text", textToSpeechSayTextHandler);
    printMsg(INFO_MSG,tabletEventPromptAndGetResponseServer.getService()+" Ready");

    ros::ServiceServer textToSpeechSayTextServer = nh.advertiseService("/tabletEvent/prompt_and_get_response", tabletEventPromptAndGetResponseHandler);
    printMsg(INFO_MSG,textToSpeechSayTextServer.getService()+" Ready");

    ros::spin();
    return 0;
}