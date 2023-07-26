/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Range reading with Pepper's back sonar sensor
 *  @file back_sonar.cpp
 *  @details The program subscribes to the topic /naoqi_driver/sonar/back which publishes 
 *           a single range from the back sonar sensor that emits energy and reports
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


class BackSonarTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/sonar/back
     * 
     */

    public:

        // constructor
        BackSonarTest()
        {
            // Create a subscriber object to get back sonar sensor data
            sub = n.subscribe("/naoqi_driver/sonar/back", 1, &BackSonarTest::backSonarCallBack, this);
        }

        void backSonarCallBack(const sensor_msgs::Range& msg)
        {
            ROS_INFO_STREAM(" ------------ Printing back sonar sensor data -----------\n\n");
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
    ros::init(argc, argv, "back_sonar_test");

    // instanciate an object from the class BackSonarTest
    BackSonarTest backSonarSensor; 

    // let ROS take over
    ros::spin();
}