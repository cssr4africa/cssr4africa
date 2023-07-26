/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Range reading with Pepper's front sonar sensor
 *  @file front_sonar.cpp
 *  @details The program subscribes to the topic /naoqi_driver/sonar/front which publishes 
 *           a single range from the front sonar sensor that emits energy and reports
 *           one range reading that is valid along an arc at the distance measured. 
 *           This message also can represent a fixed-distance (binary) ranger.
 *           This sensor will have min_range===max_range===distance of detection.
 *           These sensors follow REP 117 and will output -Inf if the object is detected
 *           and +Inf if the object is outside of the detection range.
 *           Compact message description: 
 *              message type: 
 *                  sensor_msgs/Range
 *              message content:
 *                      Header header   
 *                      uint8 ULTRASOUND            = 0 
 *                      uint8 INFRARED              = 1     
 *                      uint8 radiation_type        = 2  // the type of radiation used by the sensor (sound, IR, etc) [enum]
 *                      float32 field_of_view       = 3  // the size of the arc that the distance reading is valid for [rad]
 *                      float32 min_range           = 0  // minimum range value [m]
 *                      float32 max_range           = 1  // maximum range value [m]
 *                      float32 range                    // range data [m]  (Note: values < range_min or > range_max should be discarded)
 *  @author Pamely ZANTOU
 *  @version 1.0 
 *  @date July 2022
 * 
 *  Audit Trail
 *  -----------
 * 
 * 
 * ********************************************************************************************************************************************/




#include <ros/ros.h>
#include <sensor_msgs/Range.h>


class FrontSonarTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/sonar/front
     * 
     */
    public:

        // constructor
        FrontSonarTest()
        {
            // Create a subscriber object to get front sonar sensor data
            sub = n.subscribe("/naoqi_driver/sonar/front", 1, &FrontSonarTest::frontSonarCallBack, this);

        }

        void frontSonarCallBack(const sensor_msgs::Range& msg)
        {
             ROS_INFO_STREAM(" ------------ Printing front sonar sensor data -----------\n\n");
             ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );
             ROS_INFO_STREAM("Field of view: " << msg.field_of_view << "\n" );
             ROS_INFO_STREAM("Minimum range: " << msg.min_range << "\n" );
             ROS_INFO_STREAM("Maximum range: " << msg.max_range << "\n" );
             ROS_INFO_STREAM("---------------------------------------------------------\n\n"); 
        }
    private:

        // Create a node
        ros::NodeHandle n;

        // create subscriber object
        ros::Subscriber sub;


};


int main(int argc, char** argv)
{
    // Initialize the ROS system 
    ros::init(argc, argv, "front_sonar_test");

    // instanciate an object from the class FrontSonarTest
    FrontSonarTest frontSonarSensor; 

    // let ROS take over
    ros::spin();

    return 0;
}