/**************************************************************************************************************
 *  Pepper diagnostic routines: Sensing Pepper's hand touch
 *  @file handTouch.cpp
 *  @details The program subscribes to the topic /naoqi_driver/hand_touch which publishes 
 *                a message for Pepper's tactile interface (touch button on the hand). 
 *                Compact message description: 
 *                  message type: 
 *                      naoqi_bridge_msgs/HandTouch
 *                  message content:
 *                      uint8 hand          // which hand (left or right)
 *                      uint8 state         // state of the hands (pressed or released)
 *                      uint8 RIGHT_BACK    = 0 
 *                      uint8 RIGHT_LEFT    = 1
 *                      uint8 RIGHT_RIGHT   = 2
 *                      uint8 LEFT_BACK     = 3
 *                      uint8 LEFT_LEFT     = 4
 *                      uint8 LEFT_RIGHT    = 5
 *                      uint8 stateReleased = 0
 *                      uint8 statePressed  = 1
 *  @author Pamely ZANTOU
 *  @version 1.0 
 *  @date July 2022
 * 
 *  Audit Trail
 *  -----------
 * 
 * 
 * ***********************************************************************************************************/





#include <ros/ros.h>
#include <naoqi_bridge_msgs/HandTouch.h>


class HandTouchTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/hand_touch
     * 
     */

    public:
        HandTouchTest()
        {
            // Create a subscriber object to get hand touch data
            sub = n.subscribe("/naoqi_driver/hand_touch", 1, &HandTouchTest::handTouchCallBack, this);
        }

        void handTouchCallBack(const naoqi_bridge_msgs::HandTouch& msg)
        {
            ROS_INFO_STREAM(" ------------ Printing hand touch sensor data -----------\n\n");

            ROS_INFO_STREAM("Which hand (left or right): " << msg.hand << "\n" );

            ROS_INFO_STREAM("State of the hands (pressed or released): " << msg.state << "\n" );

            ROS_INFO_STREAM("State released value: " << msg.stateReleased << "\n" );

            ROS_INFO_STREAM("---------------------------------------------------------\n\n");
        }


    private:

        // create a node
        ros::NodeHandle n;

        // create subscriber object
        ros::Subscriber sub;

};


int main(int argc, char** argv)
{
    // Initialize the ROS system 
    ros::init(argc, argv, "hand_touch");

    // instanciate an object from the class HeadTouchTest
    HandTouchTest handTouch;

    // let ros take over
    ros::spin();

    return 0;
}
