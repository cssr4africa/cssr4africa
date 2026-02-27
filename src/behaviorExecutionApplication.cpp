// #include "behaviortree_cpp/bt_factory.h"

// file that contains the custom nodes definitions
#include "behavior_control/action_nodes.h"

/* Configuration variables */
std::string missionLanguage;
/*************************/

ros::NodeHandle* nh = nullptr;

int main(int argc, char** argv)
{  

    ros::init(argc, argv, "behaviorController");
    // BT::Tree tree;
    ros::NodeHandle nodeHandle;
    nh = &nodeHandle;

    ros::AsyncSpinner spinner(4); // Use 4 threads for the spinner
    spinner.start();

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<QueryRagNode>("QueryRagNode");
    factory.registerNodeType<QueryRagNodeSync>("QueryRagNodeSync");
    factory.registerNodeType<GetIntentNode>("GetIntentNode");
    factory.registerNodeType<isRobotSpeakingNode>("isRobotSpeakingNode");
    factory.registerNodeType<SleepNode>("SleepNode");
    factory.registerNodeType<FailureNode>("FailureNode");
    factory.registerNodeType<NavigatePortsNodeAsync>("NavigatePortsNodeAsync");
    factory.registerNodeType<IsIntentPositiveNode>("IsIntentPositiveNode");
    factory.registerNodeType<IsTourNotCompleteTree>("IsTourNotCompleteTree");
    factory.registerNodeType<SayExhibitInfoNode>("SayExhibitInfoNode");
    factory.registerNodeType<SayTextNode>("SayTextNode");
    factory.registerNodeType<RecognizeSpeechNodeAsync>("RecognizeSpeechNodeAsync");
    factory.registerNodeType<RecognizeSpeechNode>("RecognizeSpeechNode");
    factory.registerNodeType<EnableASRNode>("EnableASRNode");
    factory.registerNodeType<RetrieveListOfExhibits>("RetrieveListOfExhibits");
    factory.registerNodeType<LoadExhibitInfoNode>("LoadExhibitInfoNode");
    factory.registerNodeType<PrintExhibitInfoNode>("PrintExhibitInfoNode");
    factory.registerNodeType<SetOvertAttentionModeNode>("SetOvertAttentionModeNode");
    factory.registerNodeType<PerformGestureSync>("PerformGestureSync");
    factory.registerNodeType<NavigateNodeSync>("NavigateNodeSync");
    factory.registerNodeType<SetRobotPoseNode>("SetRobotPoseNode");
    factory.registerNodeType<NavigateNodeAsync>("NavigateNodeAsync");
    factory.registerNodeType<PerformGestureNodeAsync>("PerformGestureNodeAsync");

    /* Get the currently set language from the knowledge base*/
    missionLanguage = getMissionLanguage();

    auto tree = factory.createTreeFromFile("/home/roboticslab/workspace/pepper_rob_ws/src/behavior_control/data/my_tree.xml");
    
    std::string nodeName = ros::this_node::getName().substr(1);
    std::string softwareVersion = "1.0";
    std::string leftIndent = "\n\t\t";
    ROS_INFO_STREAM(nodeName<<": startup");
    ROS_INFO_STREAM("\n"
         + leftIndent + nodeName + "\t" + softwareVersion
         + leftIndent + "Copyright (C) 2025 Upanzi Network"
         + leftIndent + "This project is funded by the Upanzi Network"
         + leftIndent + "Website: www.cssr4africa.org\n"
         + leftIndent + "This program comes with ABSOLUTELY NO WARRANTY.\n"
    );

    // while(ros::ok())
    // {
    //     tree.tickWhileRunning();
    //     ros::spinOnce();
    //     ROS_INFO_STREAM(nodeName<<": running");
    // }

    // while (ros::ok()) 
    // {
    //     tree.tickExactlyOnce(); // Ticks the tree exactly once
    //     ROS_INFO_STREAM(nodeName<<": running");
        
    //     ros::Duration(0.01).sleep();     // Optional: Prevent 100% CPU usage
    // }

    tree.tickWhileRunning();
    // ros::spinOnce();
    ROS_INFO_STREAM(nodeName<<": running");
    return 0;
}