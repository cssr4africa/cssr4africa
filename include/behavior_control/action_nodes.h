#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behavior_control/speechEventSetEnabled.h>
#include <behavior_control/ragPrompt.h>
#include <behavior_control/overtAttentionSetMode.h>
#include <behavior_control/robotNavigationSetGoal.h>
#include <behavior_control/robotLocalizationSetPose.h>
#include <behavior_control/gestureExecutionPerformGesture.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behavior_control/QueryAction.h>
#include <behavior_control/TTSAction.h>
#include <behavior_control/ASRAction.h>
#include <behavior_control/gestureAction.h>
#include <behavior_control/setGoalAction.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <string>

#include "behavior_control/cultureKnowledgeBaseInterface.h"
#include "behavior_control/environmentKnowledgeBaseInterface.h"

extern std::string missionLanguage;

//Node Handler
extern ros::NodeHandle* nh;

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

std::string NODE_NAME = "behavior_controller";
std::string GET_INTENT_SERVICE = "/conversationManagement/get_intent";
std::string PROMPT_SERVICE = "/conversationManagement/prompt";


bool isSpeaking = false; // Global variable to know if text to speech is running or not, to avoid overlapping speech

bool isNavigating = false; // Global variable to know if navigation is running or not, to avoid overlapping navigation goals

bool verbose = false; // Global variable to control verbose output for debugging

/* Returns the current language from the knowledge base*/
std::string getMissionLanguage()
{
    strcpy(key, "phraseLanguage");
    culturalKnowledgeBase.getValue(key, &cultureKeyValue);
    return cultureKeyValue.alphanumericValue;
}

class QueryRagNode : public BT::StatefulActionNode
{
public:
    QueryRagNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), rag_client_("rag_action_server", true)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("query"),
                 BT::OutputPort<std::string>("response") };
    }

    BT::NodeStatus onStart() override
    {
        // print a message when started
        std::cout << "QueryRagNode started!" << std::endl;

        BT::Expected<std::string> query = getInput<std::string>("query");
        if (!query)
        {
            throw BT::RuntimeError("Missing required input [query]: ", query.error());
        }

        ROS_INFO("Waiting for action server: rag_action_server");
        // Wait indefinitely for the server to become available
        rag_client_.waitForServer(); 
        ROS_INFO("Action server connected!");

        // create goal, send goal, etc.
        behavior_control::QueryGoal goal;
        goal.query = query.value();
        rag_client_.sendGoal(goal);

        // print a message indicating the goal has been sent
        std::cout << "QueryRagNode sent goal: " << goal.query << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "QueryRagNode is running, checking status..." << std::endl;
        }

        if (rag_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = rag_client_.getState();
            behavior_control::QueryResultConstPtr result_ptr = rag_client_.getResult();

            ROS_INFO("Action finished: %s", result_ptr->response.c_str());

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                setOutput("response", result_ptr->response);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "QueryRagNode action status: " << rag_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "QueryRagNode halted!" << std::endl;
    }

private:
    // std::chrono::steady_clock::time_point start_time_;
    actionlib::SimpleActionClient<behavior_control::QueryAction> rag_client_;

};

class QueryRagNodeSync : public BT::SyncActionNode
{
public:
    QueryRagNodeSync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("query"),
                 BT::OutputPort<std::string>("response") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> query = getInput<std::string>("query");
        if (!query)
        {
            throw BT::RuntimeError("Missing required input [query]: ", query.error());
        }

        ros::ServiceClient rag_client = nh->serviceClient<behavior_control::ragPrompt>(PROMPT_SERVICE);
        behavior_control::ragPrompt srv;
        srv.request.prompt = query.value();
        if (rag_client.call(srv))
        {
            ROS_INFO("RAG Response: %s", srv.response.response.c_str());
            setOutput("response", srv.response.response);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /conversationManagement/prompt");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class GetIntentNode: public BT::SyncActionNode
{
public:
    GetIntentNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("query"),
                 BT::OutputPort<std::string>("response") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> query = getInput<std::string>("query");
        if (!query)
        {
            throw BT::RuntimeError("Missing required input [query]: ", query.error());
        }

        ros::ServiceClient rag_client = nh->serviceClient<behavior_control::ragPrompt>(GET_INTENT_SERVICE);
        behavior_control::ragPrompt srv;
        srv.request.prompt = query.value();
        if (rag_client.call(srv))
        {
            ROS_INFO("RAG Intent Response: %s", srv.response.response.c_str());
            setOutput("response", srv.response.response);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /conversationManagement/get_intent");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class IsIntentPositiveNode: public BT::ConditionNode
{
public:
    IsIntentPositiveNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("intent") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> intent = getInput<std::string>("intent");
        if (!intent)
        {
            throw BT::RuntimeError("Missing required input [intent]: ", intent.error());
        }

        std::string intent_value = intent.value();
        ROS_INFO("Evaluating intent: %s", intent_value.c_str());

        // Check if the intent contains the keyword "positive" (case-insensitive)
        std::transform(intent_value.begin(), intent_value.end(), intent_value.begin(), ::tolower);
        if (intent_value.find("positive") != std::string::npos)
        {
            ROS_INFO("Intent is positive.");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("Intent is negative.");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class FailureNode : public BT::SyncActionNode
{
public:
    FailureNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ROS_WARN("FailureNode ticked: returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }
};

class SleepNode : public BT::SyncActionNode
{
public:
    SleepNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("duration_ms") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<int> duration_ms = getInput<int>("duration_ms");
        if (!duration_ms)
        {
            throw BT::RuntimeError("Missing required input [duration_ms]: ", duration_ms.error());
        }

        ROS_INFO("Sleeping for %d milliseconds...", duration_ms.value());
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms.value()));
        ROS_INFO("Sleep completed.");

        return BT::NodeStatus::SUCCESS;
    }
};

class isRobotSpeakingNode : public BT::ConditionNode
{
public:
    isRobotSpeakingNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        if (isSpeaking)
        {
            ROS_INFO("Robot is currently speaking.");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("Robot is not speaking.");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class SayTextNode : public BT::StatefulActionNode
{
public:
    SayTextNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)//, tts_client_("tts_action", true)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("text") };
    }

    BT::NodeStatus onStart() override
    {
        // print a message when started
        std::cout << "SayTextNode started!" << std::endl;

        BT::Expected<std::string> text = getInput<std::string>("text");
        if (!text)
        {
            throw BT::RuntimeError("Missing required input [text]: ", text.error());
        }

        // Create the client only when the node is actually called
        if (!tts_client_) {
            tts_client_ = std::make_unique<actionlib::SimpleActionClient<behavior_control::TTSAction>>("tts_action", true);
        }

        ROS_INFO("Waiting for action server: tts_action");
        tts_client_->waitForServer(); 
        ROS_INFO("Action server connected!");

        // create goal, send goal, etc.
        behavior_control::TTSGoal goal;
        goal.text = text.value();
        tts_client_->sendGoal(goal);

        // print a message indicating the goal has been sent
        std::cout << "SayTextNode sent goal: " << goal.text << std::endl;

        // ROS_INFO("DEBUG: onStart called for goal: %s", goal.text.c_str());
        isSpeaking = true; // Set the global variable to indicate that speech is in progress

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto state = tts_client_->getState();
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "SayTextNode is running, checking status..." << std::endl;
        }

        if (state.isDone())
        {
            // Get the result pointer
            // actionlib::SimpleClientGoalState state = tts_client_.getState();
            behavior_control::TTSResultConstPtr result_ptr = tts_client_->getResult();

            ROS_INFO("Action finished!!");
            isSpeaking = false; // Reset the global variable when action is done

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Text-to-Speech succeeded: %s", result_ptr->message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "SayTextNode action status: " << state.toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "SayTextNode halted!" << std::endl;
    }

    private:
        // actionlib::SimpleActionClient<behavior_control::TTSAction> tts_client_;
        std::unique_ptr<actionlib::SimpleActionClient<behavior_control::TTSAction>> tts_client_;
};

class SayExhibitInfoNode : public BT::StatefulActionNode
{
public:
    SayExhibitInfoNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), tts_client_("tts_action", true)
    {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        // print a message when started
        std::cout << "SayExhibitInfoNode started!" << std::endl;

        // Retrieve exhibit information from blackboard
        std::string exhibit_info;
        if (!config().blackboard->rootBlackboard()->get("exhibit_description", exhibit_info))
        {
            ROS_ERROR("Failed to get current_exhibit_info from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("Waiting for action server: tts_action_server");
        // Wait indefinitely for the server to become available
        tts_client_.waitForServer(); 
        ROS_INFO("Action server connected!");

        // create goal, send goal, etc.
        behavior_control::TTSGoal goal;
        goal.text = exhibit_info;
        tts_client_.sendGoal(goal);

        // print a message indicating the goal has been sent
        std::cout << "SayExhibitInfoNode sent goal: " << goal.text << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "SayExhibitInfoNode is running, checking status..." << std::endl;
        }

        if (tts_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = tts_client_.getState();
            behavior_control::TTSResultConstPtr result_ptr = tts_client_.getResult();

            ROS_INFO("Action finished!!");

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Text-to-Speech succeeded: %s", result_ptr->message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "SayExhibitInfoNode action status: " << tts_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "SayExhibitInfoNode halted!" << std::endl;
    }

    private:
        actionlib::SimpleActionClient<behavior_control::TTSAction> tts_client_;
};

class RecognizeSpeechNode : public BT::SyncActionNode
{
    public:
    RecognizeSpeechNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), isResponseReceived(false)
    {
        /* Define a subscriber */
        subscriber = nh->subscribe("/speechEvent/text", 5, &RecognizeSpeechNode::callback, this);
    };

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("recognized_text") };
    }

    BT::NodeStatus tick() override
    {
        ros::Rate rate(1);
        ros::Time startTime = ros::Time::now();
        isResponseReceived = false;

        while(!isResponseReceived && ros::ok()){
            ros::spinOnce();

            // Timeout after 10 seconds
            if((ros::Time::now() - startTime).toSec() > 20.0){
                ROS_INFO("No speech recognized within timeout period.");
                setOutput("recognized_text", "NULL_RESPONSE");
                isResponseReceived = false;
                return BT::NodeStatus::FAILURE;
            }

            rate.sleep();
        }

        if(!ros::ok()){
            ROS_INFO("ROS is shutting down.");
            setOutput("recognized_text", "NULL_RESPONSE");
            isResponseReceived = false;
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("Recognized Speech: %s", recognizedText.c_str());
        setOutput("recognized_text", recognizedText);
        isResponseReceived = false;
        
        return BT::NodeStatus::SUCCESS;
    }

    private:
        void callback(const std_msgs::String::ConstPtr &msg)
        {
            recognizedText = msg->data;
            isResponseReceived = true;
        }

        std::string recognizedText;
        bool isResponseReceived;
        ros::Subscriber subscriber;
};

class RecognizeSpeechNodeAsync : public BT::StatefulActionNode
{
    public:
    RecognizeSpeechNodeAsync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), asr_client_("/speechEvent/recognise_speech_action", true)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("recognized_text") };
    }

    BT::NodeStatus onStart() override
    {
        // print a message when started
        std::cout << "RecognizeSpeech started!" << std::endl;

        ROS_INFO("Waiting for action server: speechEvent_action_server");
        // Wait indefinitely for the server to become available
        asr_client_.waitForServer(); 
        ROS_INFO("Speech Recognition Action server connected!");

        // create goal, send goal, etc.
        behavior_control::ASRGoal goal;
        goal.wait = 10.0; // wait for 5 seconds
        asr_client_.sendGoal(goal);

        // print a message indicating the goal has been sent
        std::cout << "RecognizeSpeechNodeAsync: waiting " << goal.wait << " seconds for speaker" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "RecognizeSpeechNodeAsync is running, checking status..." << std::endl;
        }

        if (asr_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = asr_client_.getState();
            behavior_control::ASRResultConstPtr result_ptr = asr_client_.getResult();

            ROS_INFO("Action finished!!");

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Speech Recognition succeeded: %s", result_ptr->transcription.c_str());
                // Check if the transcription is empty or not
                if (result_ptr->transcription.empty())
                {
                    ROS_WARN("Speech Recognition succeeded but no speech was recognized.");
                    setOutput("recognized_text", "NEGATIVE");
                    return BT::NodeStatus::SUCCESS;
                }
                setOutput("recognized_text", result_ptr->transcription);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                setOutput("recognized_text", "NEGATIVE");
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "RecognizeSpeechNodeAsync action status: " << asr_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "RecognizeSpeechNodeAsync halted!" << std::endl;
    }

    private:
        actionlib::SimpleActionClient<behavior_control::ASRAction> asr_client_;
};

class EnableASRNode : public BT::SyncActionNode
{
    public:
    EnableASRNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("enable") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> enable = getInput<std::string>("enable");
        if (!enable)
        {
            throw BT::RuntimeError("Missing required input [enable]: ", enable.error());
        }

        ros::ServiceClient asr_client = nh->serviceClient<behavior_control::speechEventSetEnabled>("/speechEvent/set_enabled");
        behavior_control::speechEventSetEnabled srv;
        srv.request.status = enable.value();

        if (asr_client.call(srv))
        {
            if (!srv.response.response) {
                ROS_INFO("Failed to set ASR status to %s", enable.value().c_str());
                return BT::NodeStatus::FAILURE;
            }

            ROS_INFO("ASR %s", (enable.value()=="true") ? "enabled" : "disabled");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /speechEvent/set_enabled");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class SetOvertAttentionModeNode : public BT::SyncActionNode
{
    public:
    SetOvertAttentionModeNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("mode") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> mode = getInput<std::string>("mode");
        if (!mode)
        {
            throw BT::RuntimeError("Missing required input [mode]: ", mode.error());
        }

        ros::ServiceClient overt_attention_client = nh->serviceClient<behavior_control::overtAttentionSetMode>("/overtAttention/set_mode");
        behavior_control::overtAttentionSetMode srv;
        srv.request.state = mode.value();

        if(mode.value() == "location")
        {
            Environment::GestureTargetType gesture_target;

            config().blackboard->rootBlackboard()->get("gesture_target", gesture_target);

            srv.request.location_x = gesture_target.x;
            srv.request.location_y = gesture_target.y;
            srv.request.location_z = gesture_target.z;
        }

        if (overt_attention_client.call(srv))
        {
            if (srv.response.mode_set_success == 0) {
                ROS_INFO("Failed to set Overt Attention mode to %s", mode.value().c_str());
                return BT::NodeStatus::FAILURE;
            }
            ROS_INFO("Overt Attention set to: %s", mode.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /overtAttention/set_mode");
            return BT::NodeStatus::FAILURE;
        }
        
    }
};

class PerformGestureSync : public BT::SyncActionNode
{
    public:
    PerformGestureSync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("gesture_type"), BT::InputPort<std::string>("gesture_name") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> gesture_type = getInput<std::string>("gesture_type");
        if (!gesture_type)
        {
            throw BT::RuntimeError("Missing required input [gesture_type]: ", gesture_type.error());
        }

        ros::ServiceClient gesture_client = nh->serviceClient<behavior_control::gestureExecutionPerformGesture>("/gestureExecution/perform_gesture");
        behavior_control::gestureExecutionPerformGesture srv;
        srv.request.gesture_type = gesture_type.value();

        if(gesture_type.value() == "deictic")
        {
            srv.request.gesture_id = 01;
            srv.request.gesture_duration = 3000;
            srv.request.bow_nod_angle = 0;
            Environment::GestureTargetType gestureTarget;
            std::string gestureArm;
            bool found_gesture_target = config().blackboard->get("gesture_target", gestureTarget);
            bool found_gesture_arm = config().blackboard->get("gesture_arm", gestureArm);

            if(!found_gesture_target || !found_gesture_arm)
            {
                ROS_ERROR("Missing gesture target or arm for deictic gesture.");
                return BT::NodeStatus::FAILURE;
            }

            srv.request.location_x = gestureTarget.x;
            srv.request.location_y = gestureTarget.y;
            srv.request.location_z = gestureTarget.z;
            srv.request.arm = gestureArm;
        }
        else if(gesture_type.value() == "iconic")
        {
            BT::Expected<std::string> gesture_name = getInput<std::string>("gesture_name");
            if(!gesture_name)
            {
                throw BT::RuntimeError("Missing required input [gesture_name]: ", gesture_name.error());
            }

            if(gesture_name.value() == "welcome")
            {
                srv.request.gesture_id = 01;
            }
            else if(gesture_name.value() == "goodbye")
            {
                srv.request.gesture_id = 03;
            }
            else
            {
                ROS_ERROR("Unknown iconic gesture name: %s", gesture_name.value().c_str());
                return BT::NodeStatus::FAILURE;
            }

            srv.request.gesture_duration = 3000;
            srv.request.bow_nod_angle = 0;
            srv.request.location_x = 0;
            srv.request.location_y = 0;
            srv.request.location_z = 0;
        }
        else
        {
            ROS_ERROR("Unknown gesture type: %s", gesture_type.value().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (gesture_client.call(srv))
        {
            if (srv.response.response == 0) {
                ROS_INFO("Failed to perform gesture: %s", gesture_type.value().c_str());
                return BT::NodeStatus::FAILURE;
            }
            ROS_INFO("Gesture performed: %s", gesture_type.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /gestureExecution/perform_gesture");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class PerformGestureNodeAsync : public BT::StatefulActionNode
{
    public:
    PerformGestureNodeAsync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), gesture_client_("/gestureExecution/gesture_action", true)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("gesture_type"), BT::InputPort<std::string>("gesture_name") };
    }

    BT::NodeStatus onStart() override
    {
        // print a message when started
        std::cout << "PerformGestureNode started!" << std::endl;

        ROS_INFO("Waiting for action server: gesture_execution_action_server");
        // Wait indefinitely for the server to become available
        gesture_client_.waitForServer(); 
        ROS_INFO("Action server connected!");

        // create goal, send goal, etc.
        behavior_control::gestureGoal goal;
        
        BT::Expected<std::string> gesture_type = getInput<std::string>("gesture_type");
        if (!gesture_type)
        {
            throw BT::RuntimeError("Missing required input [gesture_type]: ", gesture_type.error());
        }
        goal.gesture_type = gesture_type.value();

        if(gesture_type.value() == "deictic")
        {
            goal.gesture_id = 01;
            goal.gesture_duration = 3000;
            goal.bow_nod_angle = 0;
            Environment::GestureTargetType gestureTarget;
            std::string gestureArm;
            bool found_gesture_target = config().blackboard->get("gesture_target", gestureTarget);
            bool found_gesture_arm = config().blackboard->get("gesture_arm", gestureArm);

            if(!found_gesture_target || !found_gesture_arm)
            {
                ROS_ERROR("Missing gesture target or arm for deictic gesture.");
                return BT::NodeStatus::FAILURE;
            }

            goal.location_x = gestureTarget.x;
            goal.location_y = gestureTarget.y;
            goal.location_z = gestureTarget.z;
            goal.arm = gestureArm;
        }
        else if(gesture_type.value() == "iconic")
        {
            BT::Expected<std::string> gesture_name = getInput<std::string>("gesture_name");
            if(!gesture_name)
            {
                throw BT::RuntimeError("Missing required input [gesture_name]: ", gesture_name.error());
            }

            if(gesture_name.value() == "welcome")
            {
                goal.gesture_id = 01;
            }
            else if(gesture_name.value() == "goodbye")
            {
                goal.gesture_id = 03;
            }
            else
            {
                ROS_ERROR("Unknown iconic gesture name: %s", gesture_name.value().c_str());
                return BT::NodeStatus::FAILURE;
            }

            goal.gesture_duration = 3000;
            goal.bow_nod_angle = 0;
            goal.location_x = 0;
            goal.location_y = 0;
            goal.location_z = 0;
        }
        else
        {
            ROS_ERROR("Unknown gesture type: %s", gesture_type.value().c_str());
            return BT::NodeStatus::FAILURE;
        }

        gesture_client_.sendGoal(goal);

        std::cout << "PerformGestureNode sent goal: " << goal.gesture_type << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "PerformGestureNode is running, checking status..." << std::endl;
        }

        if (gesture_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = gesture_client_.getState();
            behavior_control::gestureResultConstPtr result_ptr = gesture_client_.getResult();

            ROS_INFO("Action finished!!");

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Gesture execution succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "PerformGestureNode action status: " << gesture_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "PerformGestureNode halted!" << std::endl;
    }

    private:
        actionlib::SimpleActionClient<behavior_control::gestureAction> gesture_client_;
};

class NavigateNodeSync : public BT::SyncActionNode
{
    public:
    NavigateNodeSync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("NavigateNodeSync ticked....");
        Environment::RobotLocationType location;
        bool found_location = config().blackboard->get("exhibit_pose", location);
        if(!found_location)
        {
            ROS_ERROR("Missing location for navigation.");
            return BT::NodeStatus::FAILURE;
        }

        ros::ServiceClient navigation_client = nh->serviceClient<behavior_control::robotNavigationSetGoal>("/robotNavigation/set_goal");
        behavior_control::robotNavigationSetGoal srv;
        srv.request.goal_x = location.x;
        srv.request.goal_y = location.y;
        srv.request.goal_theta = location.theta;

        if (navigation_client.call(srv))
        {
            if (srv.response.response == 0) {
                ROS_INFO("Failed to set navigation goal to x: %.2f, y: %.2f, theta: %.2f", 
                    location.x, location.y, location.theta);
                return BT::NodeStatus::FAILURE;
            }
            ROS_INFO("Navigation goal set to x: %.2f, y: %.2f, theta: %.2f", 
                location.x, location.y, location.theta);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to call service /robotNavigation/set_goal");
            return BT::NodeStatus::FAILURE;
        }
    }
};

class NavigateNodeAsync : public BT::StatefulActionNode
{
    public:
    NavigateNodeAsync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), navigation_client_("/robotNavigation/set_goal", true)
    {};

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        ROS_INFO("NavigateNodeAsync started....");
        Environment::RobotLocationType location;
        bool found_location = config().blackboard->get("exhibit_pose", location);
        if(!found_location)
        {
            ROS_ERROR("Missing location for navigation.");
            return BT::NodeStatus::FAILURE;
        }

        // Wait indefinitely for the server to become available
        ROS_INFO("Waiting for action server: robotNavigation_action_server");
        navigation_client_.waitForServer(); 
        ROS_INFO("Navigation Action server connected!");

        behavior_control::setGoalGoal goal;

        goal.goal_x = location.x;
        goal.goal_y = location.y;
        goal.goal_theta = location.theta;

        navigation_client_.sendGoal(goal);

        std::cout << "NavigateNodeAsync sent goal: x=" << goal.goal_x << ", y=" << goal.goal_y << ", theta=" << goal.goal_theta << std::endl;

        isNavigating = true; // Set the global variable to indicate that navigation is in progress

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "NavigateNodeAsync is running, checking status..." << std::endl;
        }

        if (navigation_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = navigation_client_.getState();
            behavior_control::setGoalResultConstPtr result_ptr = navigation_client_.getResult();

            ROS_INFO("Action finished!!");

            isNavigating = false; // Reset the global variable when action is done

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Navigation succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "NavigateNodeAsync action status: " << navigation_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "NavigateNodeAsync halted!" << std::endl;
    }

    private:
        actionlib::SimpleActionClient<behavior_control::setGoalAction> navigation_client_;
};

class NavigatePortsNodeAsync : public BT::StatefulActionNode
{
    public:
    NavigatePortsNodeAsync(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), navigation_client_("/robotNavigation/set_goal", true)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<float>("goal_x"), 
            BT::InputPort<float>("goal_y"), 
            BT::InputPort<float>("goal_theta") };
    }

    BT::NodeStatus onStart() override
    {
        ROS_INFO("NavigatePortsNodeAsync started....");
        BT::Expected<float> goal_x = getInput<float>("goal_x");
        if (!goal_x)
        {
            throw BT::RuntimeError("Missing required input [goal_x]: ", goal_x.error());
        }

        BT::Expected<float> goal_y = getInput<float>("goal_y");
        if (!goal_y)
        {
            throw BT::RuntimeError("Missing required input [goal_y]: ", goal_y.error());
        }

        BT::Expected<float> goal_theta = getInput<float>("goal_theta");
        if (!goal_theta)
        {
            throw BT::RuntimeError("Missing required input [goal_theta]: ", goal_theta.error());
        }

        // Wait indefinitely for the server to become available
        navigation_client_.waitForServer(); 
        ROS_INFO("Navigation Action server connected!");

        behavior_control::setGoalGoal goal;

        goal.goal_x = goal_x.value();
        goal.goal_y = goal_y.value();
        goal.goal_theta = goal_theta.value();

        navigation_client_.sendGoal(goal);

        std::cout << "NavigatePortsNodeAsync sent goal: x=" << goal.goal_x << ", y=" << goal.goal_y << ", theta=" << goal.goal_theta << std::endl;

        isNavigating = true; // Set the global variable to indicate that navigation is in progress

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // print a message indicating we are checking the status if verbose is true
        if (verbose)
        {
            std::cout << "NavigatePortsNodeAsync is running, checking status..." << std::endl;
        }

        if (navigation_client_.getState().isDone())
        {
            // Get the result pointer
            actionlib::SimpleClientGoalState state = navigation_client_.getState();
            behavior_control::setGoalResultConstPtr result_ptr = navigation_client_.getResult();

            ROS_INFO("Action finished!!");

            isNavigating = false; // Reset the global variable when action is done

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Navigation succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Action failed with state: %s", state.toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        // print a message indicating status of action if verbose is true
        if (verbose)
        {
            std::cout << "NavigatePortsNodeAsync action status: " << navigation_client_.getState().toString() << std::endl;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // print a message when halted
        std::cout << "NavigatePortsNodeAsync halted!" << std::endl;
    }

    private:
        actionlib::SimpleActionClient<behavior_control::setGoalAction> navigation_client_;
};

class SleepWhileNavigatingNode : public BT::SyncActionNode
{
    public:
    SleepWhileNavigatingNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        while (isNavigating)
        {
            ros::Duration(1.0).sleep(); // Sleep for 1 second while navigating
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class SetRobotPoseNode : public BT::SyncActionNode
{
    public:
    SetRobotPoseNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {};

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<float>("pose_x"), 
            BT::InputPort<float>("pose_y"), 
            BT::InputPort<float>("pose_theta"), 
            BT::InputPort<std::string>("nav_type") };
    }

    BT::NodeStatus tick() override
    {
        BT::Expected<float> pose_x = getInput<float>("pose_x");
        if (!pose_x)
        {
            throw BT::RuntimeError("Missing required input [pose_x]: ", pose_x.error());
        }

        BT::Expected<float> pose_y = getInput<float>("pose_y");
        if (!pose_y)
        {
            throw BT::RuntimeError("Missing required input [pose_y]: ", pose_y.error());
        }

        BT::Expected<float> pose_theta = getInput<float>("pose_theta");
        if (!pose_theta)
        {
            throw BT::RuntimeError("Missing required input [pose_theta]: ", pose_theta.error());
        }

        BT::Expected<std::string> nav_type = getInput<std::string>("nav_type");
        if (!nav_type)
        {
            throw BT::RuntimeError("Missing required input [nav_type]: ", nav_type.error());
        }

        ros::ServiceClient localization_client;

        if(nav_type.value() == "SLAM")
        {
            localization_client = nh->serviceClient<behavior_control::robotNavigationSetGoal>("/robotNavigation/set_pose");
            behavior_control::robotNavigationSetGoal navigation_srv;
            navigation_srv.request.goal_x = pose_x.value();
            navigation_srv.request.goal_y = pose_y.value();
            navigation_srv.request.goal_theta = pose_theta.value();

            if( localization_client.call(navigation_srv))
            {
                if (navigation_srv.response.response == 0) {
                    ROS_INFO("Failed to set robot pose to x: %.2f, y: %.2f, theta: %.2f", 
                        pose_x.value(), pose_y.value(), pose_theta.value());
                    return BT::NodeStatus::FAILURE;
                }
                ROS_INFO("Robot pose set to x: %.2f, y: %.2f, theta: %.2f", 
                    pose_x.value(), pose_y.value(), pose_theta.value());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Failed to call service /robotNavigation/set_pose");
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            localization_client = nh->serviceClient<behavior_control::robotLocalizationSetPose>("/robotLocalization/set_pose");
            behavior_control::robotLocalizationSetPose localization_srv;
            localization_srv.request.x = pose_x.value();
            localization_srv.request.y = pose_y.value();
            localization_srv.request.theta = pose_theta.value();

            if( localization_client.call(localization_srv))
            {
                if (!localization_srv.response.success) {
                    ROS_INFO("Failed to set robot pose to x: %.2f, y: %.2f, theta: %.2f", 
                        pose_x.value(), pose_y.value(), pose_theta.value());
                    return BT::NodeStatus::FAILURE;
                }
                ROS_INFO("Robot pose set to x: %.2f, y: %.2f, theta: %.2f", 
                    pose_x.value(), pose_y.value(), pose_theta.value());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_ERROR("Failed to call service /robotLocalization/set_pose");
                return BT::NodeStatus::FAILURE;
            }
        }
    }
};

class RetrieveListOfExhibits : public BT::SyncActionNode
{
   public:
    RetrieveListOfExhibits(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { };
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Retrieving list of exhibits from environment knowledge base...");
        /* Get the tour infomration from the enviornment knowledge base*/
        environmentKnowledgeBase.getTour(&tour);

        /* if tour.numberOfLocations is 0, there are no exhibits and the mission cannot continue*/
        if (tour.numberOfLocations == 0) {
            ROS_INFO("No exhibits found in the environment knowledge base.");
            return BT::NodeStatus::FAILURE;
        }
        config().blackboard->rootBlackboard()->set("exhibit_index", 0);
        ROS_INFO("Number of exhibits in the tour: %d", tour.numberOfLocations);
        return BT::NodeStatus::SUCCESS;
    }
};

class LoadExhibitInfoNode : public BT::SyncActionNode
{
   public:
    LoadExhibitInfoNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { };
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Loading exhibit information from environment knowledge base...");

        int currentExhibitIndex;
        config().blackboard->rootBlackboard()->get("exhibit_index", currentExhibitIndex);

        if (currentExhibitIndex >= tour.numberOfLocations) {
            ROS_INFO("All exhibits have been processed.");
            return BT::NodeStatus::FAILURE;
        }

        int exhibitId = tour.locationIdNumber[currentExhibitIndex];

        /* Retrieve exhibit information from the environment knowledge base */
        if (!environmentKnowledgeBase.getValue(exhibitId, &enviornmentKeyValue)) {
            ROS_WARN("Exhibit ID %d not found in the environment knowledge base.", exhibitId);
            return BT::NodeStatus::FAILURE;
        }

        /* Set outputs */
        config().blackboard->rootBlackboard()->set("exhibit_name", enviornmentKeyValue.robotLocationDescription);
        config().blackboard->rootBlackboard()->set("gesture_arm", enviornmentKeyValue.gestureArm);
        config().blackboard->rootBlackboard()->set("gesture_target", enviornmentKeyValue.gestureTarget);
        config().blackboard->rootBlackboard()->set("exhibit_pose", enviornmentKeyValue.robotLocation);

        if(missionLanguage == "English") {
            config().blackboard->rootBlackboard()->set("exhibit_description", std::string(enviornmentKeyValue.preGestureMessageEnglish) + " " + enviornmentKeyValue.postGestureMessageEnglish);
        }
        else if(missionLanguage == "IsiZulu") {
            config().blackboard->rootBlackboard()->set("exhibit_description", std::string(enviornmentKeyValue.preGestureMessageIsiZulu) + " " + enviornmentKeyValue.postGestureMessageIsiZulu);
        }
        else if(missionLanguage == "Kinyarwanda") {
            config().blackboard->rootBlackboard()->set("exhibit_description", std::string(enviornmentKeyValue.preGestureMessageKinyarwanda) + " " + enviornmentKeyValue.postGestureMessageKinyarwanda);
        }
        else {
            ROS_WARN("Unsupported mission language: %s. Defaulting to English.", missionLanguage.c_str());
            config().blackboard->rootBlackboard()->set("exhibit_description", std::string(enviornmentKeyValue.preGestureMessageEnglish) + " " + enviornmentKeyValue.postGestureMessageEnglish);
        }

        ROS_INFO("index: %d - Loaded exhibit ID %d: %s", currentExhibitIndex, exhibitId, enviornmentKeyValue.robotLocationDescription);

        config().blackboard->rootBlackboard()->set("exhibit_index", currentExhibitIndex + 1);
        return BT::NodeStatus::SUCCESS;
    }

};

class IsTourNotCompleteTree : public BT::ConditionNode
{
   public:
    IsTourNotCompleteTree(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { };
    }

    BT::NodeStatus tick() override
    {
        int currentExhibitIndex;
        config().blackboard->rootBlackboard()->get("exhibit_index", currentExhibitIndex);

        if (currentExhibitIndex > tour.numberOfLocations) {
            ROS_INFO("Tour is complete. All exhibits have been processed.");
            return BT::NodeStatus::FAILURE;
        }
        else {
            if(verbose) {
                ROS_INFO("Tour is not complete. Current exhibit index: %d, Total exhibits: %d", currentExhibitIndex, tour.numberOfLocations);
            }
            return BT::NodeStatus::SUCCESS;
        }
    }

};

class PrintExhibitInfoNode : public BT::SyncActionNode
{
   public:
    PrintExhibitInfoNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { };
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO("Printing exhibit information...");

        std::string exhibit_name;
        std::string exhibit_description;
        std::string exhibit_pose;
        std::string gesture_arm;
        Environment::GestureTargetType gesture_target;
        Environment::RobotLocationType robot_location;

        config().blackboard->rootBlackboard()->get("exhibit_name", exhibit_name);
        config().blackboard->rootBlackboard()->get("exhibit_description", exhibit_description);
        config().blackboard->rootBlackboard()->get("gesture_arm", gesture_arm);
        config().blackboard->rootBlackboard()->get("gesture_target", gesture_target);
        config().blackboard->rootBlackboard()->get("exhibit_pose", robot_location);

        std::string gesture_target_str = "x: " + std::to_string(gesture_target.x) + 
                                     ", y: " + std::to_string(gesture_target.y) + 
                                     ", z: " + std::to_string(gesture_target.z);
        std::string exhibit_pose_str = "x: " + std::to_string(robot_location.x) + 
                                     ", y: " + std::to_string(robot_location.y) + 
                                     ", theta: " + std::to_string(robot_location.theta);

        ROS_INFO("Exhibit Name: %s", exhibit_name.c_str());
        ROS_INFO("Exhibit Description: %s", exhibit_description.c_str());
        ROS_INFO("Exhibit Pose: %s", exhibit_pose_str.c_str());
        ROS_INFO("Gesture Arm: %s", gesture_arm.c_str());
        ROS_INFO("Gesture Target: %s", gesture_target_str.c_str());

        return BT::NodeStatus::SUCCESS;
    }

};