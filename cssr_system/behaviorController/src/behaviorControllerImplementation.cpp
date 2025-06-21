/* behaviorControllerImplementation.cpp   Source code for the implementation of the robot mission node classes and other utility functions
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

#include "behaviorController/behaviorControllerInterface.h"

/* Definitions for printMsg function */
#define INFO_MSG 0
#define WARNING_MSG 1
#define ERROR_MSG 2

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
void printMsg(int type, std::string args);

/*
Vocalizes a string using the system's speakers
*/
static void speak(std::string text);

/*
    Returns the number for words in a string
*/
int countWords(const std::string input);

/* Fetches the utility phrase from the culture knowledge base using the id and language */
std::string getUtilityPhrase(std::string phraseId, std::string language);

/*
    Stores the result of a nodes execution in the paramteter server.
    To be used by the test node.
*/
static void storeResult(std::string key, int value);

/***** Global Variables ****/
// Environment Knowledge Base
Environment::EnvironmentKnowledgeBase environmentKnowledgeBase;
Environment::TourSpecificationType tour;
Environment::KeyValueType enviornmentKeyValue;

// Culture Knowledge Base
Culture::KeyValueType cultureKeyValue;
Culture::CultureKnowledgeBase culturalKnowledgeBase;
Culture::Keyword key;
/********************************** */

/****** Mission(Action/Condition) Nodes */
/*
    Handler for the 'HandleFallBack' Action Node
*/
class HandleFallBack : public BT::SyncActionNode
{
   public:
    HandleFallBack(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "HandleFallback";
        printMsg(INFO_MSG, "(" + treeNodeName + " Action Node)");

        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetAnimateBehavior' Action Node
    Enables & Disables animate behavior
*/
class SetAnimateBehavior : public BT::SyncActionNode
{
   public:
    SetAnimateBehavior(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::animateBehaviorSetActivation>("/animateBehaviour/setActivation");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::animateBehaviorSetActivation srv;
        std::string state = name();  // retrieve the state from the mission specification
        std::string treeNodeName = "SetAnimateBehavior";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "State: " + state);
        speak(treeNodeName + " " + state);

        srv.request.state = state;
        /* Make a service call to animateBehavior node and act according to the response*/
        if (client.call(srv)) {
            if (srv.response.success != "1") {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetOvertAttentionMode' Action Node
    Set different values to the attention mode of the overtAttention ROS node
*/
class SetOvertAttentionMode : public BT::SyncActionNode
{
   public:
    SetOvertAttentionMode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::overtAttentionSetMode>("/overtAttention/set_mode");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::overtAttentionSetMode srv;
        std::string state = name();  // retrieve the state from the mission specification
        std::string treeNodeName = "SetOvertAttentionMode";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "State: " + state);
        speak(treeNodeName + " " + state);

        srv.request.state = state;

        // if mode is 'location', retrieve the target from the blackboard
        if (state == "location") {
            Environment::RobotLocationType location;
            Environment::GestureTargetType gestureTarget;
            if (!config().blackboard->rootBlackboard()->get("exhibitGestureTarget", gestureTarget)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }

            if (!(gestureTarget.x + gestureTarget.y + gestureTarget.z)) {
                return BT::NodeStatus::SUCCESS;
            }

            // Set the retrieved values in order to make a service call
            srv.request.location_x = gestureTarget.x;
            srv.request.location_y = gestureTarget.y;
            srv.request.location_z = gestureTarget.z;
        }

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.mode_set_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetSpeechEvent' Action Node
    Enables & Disables transcription on the speechEvent ROS node
*/
class SetSpeechEvent : public BT::SyncActionNode
{
   public:
    SetSpeechEvent(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::speechEventSetStatus>("/speechEvent/set_enabled");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::speechEventSetStatus srv;
        std::string status = name();  // retrieve the status from the mission specification
        std::string treeNodeName = "SetSpeechEvent";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "Status: " + status);
        speak(treeNodeName + " " + status);

        srv.request.status = status;
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.response) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};
/*
    Handler for the 'IsVisitorDiscovered' Condition Node
    Checks for the presence of a visitor via the overtAttention ROS node
*/
class IsVisitorDiscovered : public BT::ConditionNode
{
   public:
    IsVisitorDiscovered(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), visitorDiscovered(false)
    {
        /* Define a subscriber to the topic */
        subscriber = nh->subscribe("/overtAttention/mode", 10, &IsVisitorDiscovered::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsVisitorDiscovered";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        ros::Rate rate(10);
        /* Wait until the topic returns data, indicating arrival of a potential visitor */
        while (ros::ok()) {
            ros::spinOnce();

            if (visitorDiscovered) {
                printMsg(INFO_MSG, "Visitor discovered");
                storeResult(treeNodeName, 1);
                visitorDiscovered = false;
                return BT::NodeStatus::SUCCESS;
            }

            rate.sleep();
        }

        storeResult(treeNodeName, 0);
        visitorDiscovered = false;
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const cssr_system::overtAttentionMode::ConstPtr &msg)
    {
        /*
            Values 2 indicates success
        */
        visitorDiscovered = false;
        if (msg->state == "scanning" && msg->value == 2) {
            visitorDiscovered = true;
        }
    }

    bool visitorDiscovered;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'IsVisitorPresent' Condition Node
    Checks for the presence of a visitor via the overtAttention ROS node
*/
class IsVisitorPresent : public BT::ConditionNode
{
   public:
    IsVisitorPresent(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), visitorPresent(false)
    {
        /* Define a subscriber to the topic */
        subscriber = nh->subscribe("/overtAttention/mode", 10, &IsVisitorPresent::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsVisitorPresent";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        ros::Rate rate(10);
        /* Wait until the topic returns data, indicating arrival of a potential visitor */
        ros::Time startTime = ros::Time::now();
        while ((ros::Time::now() - startTime).toSec() < 10) {
            ros::spinOnce();
            if (visitorPresent) {
                printMsg(INFO_MSG, "Visitor Still Present");
                storeResult(treeNodeName, 1);
                visitorPresent = false;
                return BT::NodeStatus::SUCCESS;
            }

            rate.sleep();
        }

        storeResult(treeNodeName, 0);
        visitorPresent = false;
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const cssr_system::overtAttentionMode::ConstPtr &msg)
    {
        /*
            Values 2 indicates success
        */
        visitorPresent = false;
        if (msg->state == "social" && msg->value == 2) {
            visitorPresent = true;
        }
    }

    bool visitorPresent;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'IsMutualGazeDiscovered' Condition Node
    Checks for the detection of mutual gaze via overtAttention ROS node
*/
class IsMutualGazeDiscovered : public BT::ConditionNode
{
   public:
    IsMutualGazeDiscovered(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), seekingStatus("RUNNING")
    {
        /* Define a subscriber to the topic */
        subscriber = nh->subscribe("/overtAttention/mode", 10, &IsMutualGazeDiscovered::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsMutualGazeDiscovered";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        ros::Rate rate(10);

        /* Keep checking for the detection of mutual gaze */
        while (ros::ok()) {
            ros::spinOnce();
            if (seekingStatus == "SUCCESS") {
                printMsg(INFO_MSG, "Mutual gaze detected");
                storeResult(treeNodeName, 1);
                return BT::NodeStatus::SUCCESS;
            } else if (seekingStatus == "FAILURE") {
                printMsg(INFO_MSG, "Mutual gaze detection failed");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }

            rate.sleep();
        }

        storeResult(treeNodeName, 0);
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const cssr_system::overtAttentionMode::ConstPtr &msg)
    {
        /*
            Values 2 & 3, indicating success & failure respectively are how
            the overtAttention node relays 'seeking' mode status
        */

        if (msg->state == "seeking" && msg->value == 2) {
            seekingStatus = "SUCCESS";
        } else if (msg->state == "seeking" && msg->value == 3) {
            seekingStatus = "FAILURE";
        } else {
            seekingStatus = "RUNNING";
        }
    }

    std::string seekingStatus;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'DescribeExhibitSpeech' Action Node
    Sends exhibit description to be uttered by the textToSpeech ROS node
    The exhibit selected changes as the mission execution loops over the list of exhibits
*/
class DescribeExhibitSpeech : public BT::SyncActionNode
{
   public:
    DescribeExhibitSpeech(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::textToSpeechSayText>("/textToSpeech/say_text");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::textToSpeechSayText srv;
        srv.request.language = missionLanguage;
        std::string nodeInstance = name();  // retrieve nodeInstance value from the specification
        std::string message;
        std::string treeNodeName = "DescribeExhibit";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        /* '1' indicates Pre-Gesture Message */
        if (nodeInstance == "1") {
            if (!config().blackboard->get("exhibitPreGestureMessage", message)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else if (nodeInstance == "2") {  // '2' indicates Post-Gesture message
            if (!config().blackboard->get("exhibitPostGestureMessage", message)) {
                printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(WARNING_MSG, "Invalid Node Instance");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        srv.request.message = message;
        int wordCount = countWords(srv.request.message);
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        if (missionLanguage=="English"){
            ros::Duration(wordCount/3.0).sleep();
        }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SayText' Action Node
    Sends text to be uttered by the textToSpeech ROS node
*/
class SayText : public BT::SyncActionNode
{
   public:
    SayText(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::textToSpeechSayText>("/textToSpeech/say_text");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::textToSpeechSayText srv;
        srv.request.language = missionLanguage;
        std::string utilityPhraseId = name();  // retrieve id from the mission specification
        std::string treeNodeName = "SayText";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        srv.request.message = getUtilityPhrase(utilityPhraseId, missionLanguage);
        int wordCount = countWords(srv.request.message);
        
        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        // if (missionLanguage=="English"){
        //     ros::Duration(wordCount/3.0).sleep();
        // }
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PerformDeicticGesture' Action Node
    Performs a deictic gesture via the gestureExecution ROS node
*/
class PerformDeicticGesture : public BT::SyncActionNode
{
   public:
    PerformDeicticGesture(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::gestureExecutionPerformGesture>("/gestureExecution/perform_gesture");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::gestureExecutionPerformGesture srv;
        Environment::GestureTargetType gestureTarget;
        std::string treeNodeName = "PerformDeicticGesture";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        /* Retrieve gesture values from the blackboard*/
        if (!config().blackboard->rootBlackboard()->get("exhibitGestureTarget", gestureTarget)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        if (!(gestureTarget.x + gestureTarget.y + gestureTarget.z)) {
            return BT::NodeStatus::SUCCESS;
        }

        srv.request.gesture_type = "deictic";
        srv.request.gesture_id = 01;
        srv.request.gesture_duration = 3000;
        srv.request.bow_nod_angle = 0;
        srv.request.location_x = gestureTarget.x;
        srv.request.location_y = gestureTarget.y;
        srv.request.location_z = gestureTarget.z;

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.gesture_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PerformIconicGesture' Action Node
    Performs an iconic gesture via the gestureExecution ROS node
*/
class PerformIconicGesture : public BT::SyncActionNode
{
   public:
    PerformIconicGesture(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::gestureExecutionPerformGesture>("/gestureExecution/perform_gesture");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::gestureExecutionPerformGesture srv;
        std::string iconicGestureType = name();  // retrieve the gesture type from the mission specification
        std::string treeNodeName = "PerformIconicGesture";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        printMsg(INFO_MSG, "Gesture: " + iconicGestureType);
        speak(treeNodeName);

        srv.request.gesture_type = "iconic";
        srv.request.gesture_duration = 3000;
        srv.request.bow_nod_angle = 0;
        srv.request.location_x = 0;
        srv.request.location_y = 0;
        srv.request.location_z = 0;

        /*
            '01' and '03' represent the 'welcome' and 'goodbye'
             iconic gestures in the gestureExecution node
        */
        if (iconicGestureType == "welcome") {
            srv.request.gesture_id = 01;
        } else if (iconicGestureType == "goodbye") {
            srv.request.gesture_id = 03;
        } else {
            printMsg(ERROR_MSG, "Undefined Iconic Gesture Type");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.gesture_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'PressYesNoDialogue' Action Node
    Initiates a dialogue on the robot's table via tabletEvent ROS node
*/
class PressYesNoDialogue : public BT::SyncActionNode
{
   public:
    PressYesNoDialogue(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::tabletEventPromptAndGetResponse>("/tabletEvent/prompt_and_get_response");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "PressYesNoDialogue(Tablet)";
        cssr_system::tabletEventPromptAndGetResponse srv;
        srv.request.message = "'Yes'|'No'";

        printMsg(INFO_MSG, treeNodeName + " Action Node");
        speak(treeNodeName);

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'RetrieveListOfExhibits' Action Node
    Retrieve the exhibits for the tour from the environment knowledge base
*/
class RetrieveListOfExhibits : public BT::SyncActionNode
{
   public:
    RetrieveListOfExhibits(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "RetrieveListOfExhibits";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        /* Get the tour infomration from the enviornment knowledge base*/
        environmentKnowledgeBase.getTour(&tour);

        /* if tour.numberOfLocations is 0, there are no exhibits and the mission cannot continue*/
        if (tour.numberOfLocations == 0) {
            printMsg(ERROR_MSG, "Number of Exhibits is 0");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        config().blackboard->set("visits", 0);
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }
};

/*
    Handler for the 'SelectExhibit' Action Node
    Selects the next exhibit from the list
*/
class SelectExhibit : public BT::SyncActionNode
{
   public:
    SelectExhibit(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "SelectExhibit";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        int visits = 0;
        std::string preGestureMessage;
        std::string postGestureMessage;

        if (!config().blackboard->get("visits", visits)) {
            printMsg(WARNING_MSG, "Exhibit list empty");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        // Select the next exhibit to visit
        environmentKnowledgeBase.getValue(tour.locationIdNumber[visits], &enviornmentKeyValue);

        // Retrieve the exhibit description based on the current set language
        if (missionLanguage == "English") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageEnglish;
            postGestureMessage = enviornmentKeyValue.postGestureMessageEnglish;
        } else if (missionLanguage == "Kinyarwanda") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageKinyarwanda;
            postGestureMessage = enviornmentKeyValue.postGestureMessageKinyarwanda;
        } else if (missionLanguage == "IsiZulu") {
            preGestureMessage = enviornmentKeyValue.preGestureMessageIsiZulu;
            postGestureMessage = enviornmentKeyValue.postGestureMessageIsiZulu;
        } else {
            printMsg(ERROR_MSG, "Unknown language set");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        // Store the values in the blackboard to be retrieved by other mission nodes
        config().blackboard->set("exhibitPreGestureMessage", preGestureMessage);
        config().blackboard->set("exhibitPostGestureMessage", postGestureMessage);
        config().blackboard->rootBlackboard()->set("exhibitLocation", enviornmentKeyValue.robotLocation);
        config().blackboard->rootBlackboard()->set("exhibitGestureTarget", enviornmentKeyValue.gestureTarget);

        printMsg(INFO_MSG, std::string("Visiting: ") + enviornmentKeyValue.robotLocationDescription);

        config().blackboard->set("visits", ++visits);  // indicate that the current exhbit is already 'visitited', when checked later
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }
};

/*
    Handler for the 'IsListWithExhibit' Condition Node
    Checks if there are any more exhibits to visit
*/
class IsListWithExhibit : public BT::ConditionNode
{
   public:
    IsListWithExhibit(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsListWithExhibit";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        int visits = 0;
        if (!config().blackboard->get("visits", visits)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }

        // if the number of 'visits' value exceeds the number of locations, then all exhibits have been visited
        if (visits < tour.numberOfLocations) {
            storeResult(treeNodeName, 1);
            return BT::NodeStatus::SUCCESS;
        } else {
            printMsg(INFO_MSG, "ALL LANDMARKS VISITED");
            return BT::NodeStatus::FAILURE;
        }
    }
};

/*
    Handler for the 'Navigate' Action Node
    Performs navigation via the robotNaviggation ROS node
*/
class Navigate : public BT::SyncActionNode
{
   public:
    Navigate(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::robotNavigationSetGoal>("/robotNavigation/set_goal");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cssr_system::robotNavigationSetGoal srv;
        Environment::RobotLocationType location;
        std::string treeNodeName = "Navigate";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        // Retrieve location values fromthe blackboard
        if (!config().blackboard->rootBlackboard()->get("exhibitLocation", location)) {
            printMsg(ERROR_MSG, "Unable to retrieve from blackboard");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        srv.request.goal_x = location.x;
        srv.request.goal_y = location.y;
        srv.request.goal_theta = location.theta;

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.navigation_goal_success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'SetRobotPose' Action Node
    Resets the current pose via the robotLocalization ROS node
*/
class SetRobotPose : public BT::SyncActionNode
{
   public:
    SetRobotPose(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::robotLocalizationSetPose>("/robotLocalization/set_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        Environment::RobotLocationType location;
        cssr_system::robotLocalizationSetPose srv;
        std::string treeNodeName = "SetRobotPose";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        environmentKnowledgeBase.getValue(tour.locationIdNumber[0], &enviornmentKeyValue);
        location = enviornmentKeyValue.robotLocation;

        srv.request.x = location.x;
        srv.request.y = location.y;
        srv.request.theta = location.theta;

        /* Make a service call to the node and act according to the response*/
        if (client.call(srv)) {
            if (!srv.response.success) {
                printMsg(WARNING_MSG, "Called service returned failure");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        } else {
            printMsg(ERROR_MSG, "Failed to call service");
            storeResult(treeNodeName, 0);
            return BT::NodeStatus::FAILURE;
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
};

/*
    Handler for the 'RetrieveInitialLocation' Action Node
    Retrieves the initial location and sets it in the blackboard
*/
class RetrieveInitialLocation : public BT::SyncActionNode
{
   public:
    RetrieveInitialLocation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        Environment::RobotLocationType location;
        cssr_system::robotLocalizationSetPose srv;
        std::string treeNodeName = "RetrieveInitialLocation";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);

        environmentKnowledgeBase.getValue(tour.locationIdNumber[0], &enviornmentKeyValue);
        location = enviornmentKeyValue.robotLocation;

        srv.request.x = location.x;
        srv.request.y = location.y;
        srv.request.theta = location.theta;

        config().blackboard->rootBlackboard()->set("exhibitLocation", location);

        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }
};

/*
    Handler for the 'GetVisitorResponse' Action Node
    Retrieves visitor responses via the speechEvent ROS node
*/
class GetVisitorResponse : public BT::SyncActionNode
{
   public:
    GetVisitorResponse(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), isResponseReceived(false)
    {
        subscriber = nh->subscribe("/speechEvent/text", 10, &GetVisitorResponse::callback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "GetVisitorResponse";

        printMsg(INFO_MSG, treeNodeName + "Action Node");
        speak(treeNodeName);
        ros::Rate rate(1);
        ros::Time startTime = ros::Time::now();
        auto blackboard = config().blackboard->rootBlackboard();
        while (ros::ok()) {
            ros::spinOnce();
            if (isResponseReceived) {
                if (visitorResponse.find("Error") != string::npos) {
                    printMsg(WARNING_MSG, "Errror encountered");
                    blackboard->set("visitorResponse", "");
                    storeResult(treeNodeName, 0);
                    isResponseReceived = false;
                    return BT::NodeStatus::FAILURE;
                }

                /** Set of words to check for an affirmative response from the visitor */
                std::map<std::string, std::vector<std::string>> affirmativeWords;
                affirmativeWords["English"] = {"yes", "great", "absolutely", "go", "happy", "good", "love"};
                affirmativeWords["Kinyarwanda"] = {"yego", "ntakibazo", "nibyo"};

                std::map<std::string, std::vector<std::string>> negativeWords;
                negativeWords["English"] = {"no"};
                negativeWords["Kinyarwanda"] = {"oya"};

                /*If any of the 'affirmative' words are detected, set 'visitorResponse' as yes in the blackboard */
                for (const string &affirmativeWord : affirmativeWords[missionLanguage]) {
                    if (visitorResponse.find(affirmativeWord) != string::npos) {
                        blackboard->set("visitorResponse", "yes");
                        printMsg(INFO_MSG, "Visitor Response: " + visitorResponse);
                        storeResult(treeNodeName, 1);
                        isResponseReceived = false;
                        return BT::NodeStatus::SUCCESS;
                    }
                }

                /*If any of the 'negative' words are detected, set 'visitorResponse' as no in the blackboard */
                for (const string &affirmativeWord : negativeWords[missionLanguage]) {
                    if (visitorResponse.find(affirmativeWord) != string::npos) {
                        blackboard->set("visitorResponse", "no");
                        printMsg(WARNING_MSG, "Visitor Response: " + visitorResponse);
                        storeResult(treeNodeName, 1);
                        isResponseReceived = false;
                        return BT::NodeStatus::SUCCESS;
                    }
                }

                
            }

            /* If there is no response within 10 seconds, set 'visitorResponse' as empty in the blackboard*/
            if ((ros::Time::now() - startTime).toSec() > 10) {
                printMsg(WARNING_MSG, "No affirmative response received");
                blackboard->set("visitorResponse", "");
                storeResult(treeNodeName, 0);
                isResponseReceived = false;
                return BT::NodeStatus::SUCCESS;
            }

            rate.sleep();
        }
        storeResult(treeNodeName, 0);
        isResponseReceived = false;
        return BT::NodeStatus::FAILURE;
    }

   private:
    void callback(const std_msgs::String::ConstPtr &msg)
    {
        visitorResponse = msg->data;
        isResponseReceived = true;
    }

    std::string visitorResponse;
    bool isResponseReceived;
    ros::Subscriber subscriber;
};

/*
    Handler for the 'IsVisitorResponseYes' Condition Node
    Checks if the response from the visitor is affirmative
    Validates the final check to start the tour or end interaction
*/
class IsVisitorResponseYes : public BT::ConditionNode
{
   public:
    IsVisitorResponseYes(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsVisitorResponseYes";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        auto blackboard = config().blackboard->rootBlackboard();
        std::string visitorResponse;
        if (blackboard->get("visitorResponse", visitorResponse)) {
            if (visitorResponse == "yes") {
                storeResult(treeNodeName, 1);
                return BT::NodeStatus::SUCCESS;
            }
        }

        storeResult(treeNodeName, 0);
        return BT::NodeStatus::FAILURE;
    }
};

/*
    Handler for the 'HasVisitorResponded' Condition Node
    Checks if the response from the visitor is affirmative
    Checks if a visitor has given an actionable response
*/
class HasVisitorResponded : public BT::ConditionNode
{
   public:
    HasVisitorResponded(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "HasVisitorResponded";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        speak(treeNodeName);

        auto blackboard = config().blackboard->rootBlackboard();
        std::string visitorResponse;
        if (blackboard->get("visitorResponse", visitorResponse)) {
            if (visitorResponse != "") {
                storeResult(treeNodeName, 1);
                return BT::NodeStatus::SUCCESS;
            }
        }

        storeResult(treeNodeName, 0);
        return BT::NodeStatus::FAILURE;
    }
};

/*
    Handler for the 'StartOfTree' Action Node
    Used for logging debugging information if verboseMose is enabled
    Used for initializing certain mission paramters
        - the language used by speechEvent ROS node for transcription
*/
class StartOfTree : public BT::SyncActionNode
{
   public:
    StartOfTree(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), missionStarted(false), firstRun(true)
    {
        /* Define a service client */
        client = nh->serviceClient<cssr_system::speechEventSetLanguage>("/speechEvent/set_language");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "StartOfTree";
        printMsg(INFO_MSG,
                 "\n============================\n"
                 "\n\tSTART OF TREE\n"
                 "\n============================\n");
                 speak("Start of Tree");
        std::string userInput;
        if(missionStarted){
            missionStarted=false;
            storeResult("MissionEnded", 1);
        }
        if(firstRun){
            ROS_INFO_STREAM(nodeName<<": Press 'Enter' to start the mission");
            std::getline(std::cin, userInput);
            firstRun = false;
        }else{
            ROS_INFO_STREAM(nodeName<<": Do you want to run the mission again(y/n)?");
            std::getline(std::cin, userInput);
            if (userInput!="y"){
                ros::shutdown();
            }
        }
        
        missionStarted=true;
        storeResult("MissionStarted", 1);
        // As this node is executed at the start of the mission, this is where
        // variables with mission execution lifetime are set

        // Setting the language for speechEvent if ASR is enabled
        if (asrEnabled) {
            cssr_system::speechEventSetLanguage srv;
            srv.request.language = missionLanguage;

            printMsg(INFO_MSG, "ASR Enabled. Setting language to: " + missionLanguage);

            if (client.call(srv)) {
                if (!srv.response.response) {
                    printMsg(WARNING_MSG, "Called service returned failure");
                    storeResult(treeNodeName, 0);
                    return BT::NodeStatus::FAILURE;
                }
            } else {
                printMsg(ERROR_MSG, "Failed to call service");
                storeResult(treeNodeName, 0);
                return BT::NodeStatus::FAILURE;
            }
        }
        storeResult(treeNodeName, 1);
        return BT::NodeStatus::SUCCESS;
    }

   private:
    ros::ServiceClient client;
    bool missionStarted;
    bool firstRun;
};

/*
    Handler for the 'IsASREnabled' Condition Node
    Checks if ASR is set to enabled in the configuration file
*/
class IsASREnabled : public BT::ConditionNode
{
   public:
    IsASREnabled(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::string treeNodeName = "IsASREnabled";

        printMsg(INFO_MSG, treeNodeName + "Condition Node");
        if (asrEnabled) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

/*****************************************************/

BT::Tree initializeTree(std::string scenario)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<StartOfTree>("StartOfTree");
    factory.registerNodeType<DescribeExhibitSpeech>("DescribeExhibitSpeech");
    factory.registerNodeType<GetVisitorResponse>("GetVisitorResponse");
    factory.registerNodeType<HandleFallBack>("HandleFallBack");
    factory.registerNodeType<HasVisitorResponded>("HasVisitorResponded");
    factory.registerNodeType<IsASREnabled>("IsASREnabled");
    factory.registerNodeType<IsListWithExhibit>("IsListWithExhibit");
    factory.registerNodeType<IsMutualGazeDiscovered>("IsMutualGazeDiscovered");
    factory.registerNodeType<IsVisitorDiscovered>("IsVisitorDiscovered");
    factory.registerNodeType<IsVisitorPresent>("IsVisitorPresent");
    factory.registerNodeType<IsVisitorResponseYes>("IsVisitorResponseYes");
    factory.registerNodeType<Navigate>("Navigate");
    factory.registerNodeType<PerformDeicticGesture>("PerformDeicticGesture");
    factory.registerNodeType<PerformIconicGesture>("PerformIconicGesture");
    factory.registerNodeType<PressYesNoDialogue>("PressYesNoDialogue");
    factory.registerNodeType<RetrieveInitialLocation>("RetrieveInitialLocation");
    factory.registerNodeType<RetrieveListOfExhibits>("RetrieveListOfExhibits");
    factory.registerNodeType<SayText>("SayText");
    factory.registerNodeType<SelectExhibit>("SelectExhibit");
    factory.registerNodeType<SetAnimateBehavior>("SetAnimateBehavior");
    factory.registerNodeType<SetOvertAttentionMode>("SetOvertAttentionMode");
    factory.registerNodeType<SetRobotPose>("SetRobotPose");
    factory.registerNodeType<SetSpeechEvent>("SetSpeechEvent");

    return factory.createTreeFromFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorController/data/" + scenario + ".xml");
}

/***** Utility Functions ******/

/*
Logs the string (args) to the terminal based on the (type).
Wrapper around the default ROS logging functions
*/
void printMsg(int type, std::string args)
{
    if (!verboseMode) {
        return;
    }

    std::string msg = nodeName + ": " + args;
    switch (type) {
        case INFO_MSG:
            ROS_INFO_STREAM(msg);
            break;
        case WARNING_MSG:
            ROS_WARN_STREAM(msg);
            break;
        case ERROR_MSG:
            ROS_ERROR_STREAM(msg);
            break;
        default:
            ROS_ERROR_STREAM("UNDEFINED MSG TYPE");
    }
}

/* Returns the current language from the knowledge base*/
std::string getMissionLanguage()
{
    strcpy(key, "phraseLanguage");
    culturalKnowledgeBase.getValue(key, &cultureKeyValue);
    return cultureKeyValue.alphanumericValue;
}

/* Fetches the utility phrase from the culture knowledge base using the id and language */
std::string getUtilityPhrase(std::string phraseId, std::string language)
{
    std::string phraseKey = "utilityPhrase" + language + phraseId;
    strcpy(key, phraseKey.c_str());
    culturalKnowledgeBase.getValue(key, &cultureKeyValue);
    return cultureKeyValue.alphanumericValue;
}

/* Returns true if ch isn't an empty space character*/
static bool isNotSpace(unsigned char ch)
{
    return !std::isspace(ch);
}

/* Trims whitespaces inplace */
static inline void trim(std::string &s)
{
    // Trim leading spaces
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), isNotSpace));

    // Trim trailing spaces
    s.erase(std::find_if(s.rbegin(), s.rend(), isNotSpace).base(), s.end());
}

/* Returns the value of a key from the configuration file. */
std::string getValueFromConfig(const std::string &key)
{
    std::string value;
    std::ifstream configFile(ros::package::getPath(ROS_PACKAGE_NAME) + "/behaviorController/config/behaviorControllerConfiguration.ini");
    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    std::string configLineRead;

    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;

        trim(paramKey);
        std::getline(iss, paramValue);
        trim(paramValue);

        if (paramKey == key) {
            value = paramValue;
            break;
        }
    }
    configFile.close();
    if (value.empty()) {
        throw std::runtime_error("Failed to retrieve value for key: " + key);
    }
    return value;
}

/* Returns true if a topic is available */
static bool isTopicAvailable(std::string topic)
{
    ros::master::V_TopicInfo masterTopics;
    ros::master::getTopics(masterTopics);

    // Iterate through the topics to check if the topic is available
    for (const auto &topicEntry : masterTopics) {
        if (topicEntry.name == topic) {
            return true;
        }
    }
    return false;
}

/* Returns true if all the topics in a list are available*/
bool checkTopics(std::vector<std::string> &topicsList)
{
    bool success = true;
    for (std::string topic : topicsList) {
        if (!isTopicAvailable(topic)) {
            success = false;
            ROS_ERROR_STREAM("[" << topic << "] NOT FOUND");
        }
    }
    return success;
}

/* Returns true if all the services in a list are available*/
bool checkServices(std::vector<std::string> &servicesList)
{
    bool success = true;
    for (std::string service : servicesList) {
        if (!ros::service::exists(service, false)) {
            success = false;
            ROS_ERROR_STREAM("[" << service << "] NOT FOUND");
        }
    }
    return success;
}

/*
    Stores the result of a nodes execution in the paramteter server.
    To be used by the test node.
*/
static void storeResult(std::string key, int value = -1)
{
    std::string testParameterPath = "/behaviorControllerTest/";
    ros::param::set((testParameterPath + key), value);
}


/*
Vocalizes a string using the system's speakers
*/
static void speak(std::string text)
{
    if(!audioDebugMode){
        return;
    }
    std::string textToSay = "espeak '"+text+"'";
    int i = system(textToSay.c_str());
}

/*
    Returns the number for words in a string
*/
int countWords(const std::string input) {
    std::istringstream stream(input);
    std::string word;
    int count = 0;

    while (stream >> word) {
        ++count;
    }

    return count;
}

/****************************** */
