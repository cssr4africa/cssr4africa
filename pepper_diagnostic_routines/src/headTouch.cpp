/**************************************************************************************************************
 *  Pepper diagnostic routines: Sensing Pepper's head touch
 *  @file headTouch.cpp
 *  @details The program subscribes to the topic /naoqi_driver/head_touch which publishes 
 *                a message for Pepper's tactile interface (touch button on the head). 
 *                Compact message description: 
 *                  message type: 
 *                      naoqi_bridge_msgs/HeadTouch
 *                  message content:
 *                      uint8 button  // which of the three segments is touched
 *                      uint8 state   // pressed or released, see below
 *                      uint8 buttonFront   = 1 
 *                      uint8 buttonMiddle  = 2
 *                      uint8 buttonRear    = 3
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
#include <naoqi_bridge_msgs/HeadTouch.h>




class HeadTouchTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/head_touch
     * 
     */

    public:
        HeadTouchTest()
        {
            // Create a subscriber object to get head touch data
            sub = n.subscribe("/naoqi_driver/head_touch", 1, &HeadTouchTest::headTouchCallBack, this);
        }

        void headTouchCallBack(const naoqi_bridge_msgs::HeadTouch& msg)
        {
            ROS_INFO_STREAM(" ------------ Printing Head touch sensor data -----------\n\n");

            ROS_INFO_STREAM("Segments is touched: " << msg.button << "\n" );

            ROS_INFO_STREAM("State of the Heads (pressed or released): " << msg.state << "\n" );

            ROS_INFO_STREAM("Button front value: " << msg.buttonFront << "\n" );

            ROS_INFO_STREAM("Button middle value: " << msg.buttonMiddle << "\n" );

            ROS_INFO_STREAM("Button rear value: " << msg.buttonRear << "\n" );

            ROS_INFO_STREAM("State released: " << msg.stateReleased << "\n" );

            ROS_INFO_STREAM("State pressed: " << msg.statePressed << "\n" );

            ROS_INFO_STREAM("---------------------------------------------------------\n\n");   
        }


    private:

        // Create a node
        ros::NodeHandle n;
        
        // create subscriber object
        ros::Subscriber  sub;

};



int main(int argc, char **argv)
{
    // Initialize the ROS system 
    ros::init(argc, argv, "head_touch");

    // instanciate an object from the class HeadTouchTest
    HeadTouchTest headTouch;    

    // let ROS take over
    ros::spin(); 

    return 0;
}