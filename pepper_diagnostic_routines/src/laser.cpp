/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Range reading with Pepper's laser sensor
 *  @file laser.cpp
 *  @details The program subscribes to the topic /naoqi_driver/laser which publishes 
 *           a single scan from a planar laser range-finder. 
 *           Compact message description: 
 *              message type: 
 *                  sensor_msgs/LaserScan
 *              message content:
 *                      Header header                    // timestamp in the header is the acquisition time of the first ray in the scan.
 *                                                       // in frame frame_id, angles are measured around the positive Z axis 
 *                                                       // (counterclockwise, if Z is up) with zero angle being forward along the x axis   
 *                                            
 *                      float32 angle_min                // float32 angle_min
 *                      float32 angle_max                // end angle of the scan [rad]     
 *                      float32 angle_increment          // angular distance between measurements [rad]
 *                      float32 time_increment           // time between measurements [seconds] - if your scanner is moving, this will 
 *                                                       // be used in interpolating position of 3d points time between scans [seconds]
 * 
 *                      float32 range_min                // minimum range value [m]
 *                      float32 range_max                // maximum range value [m]
 *                      float32[] ranges                 // range data [m] (Note: values < range_min or > range_max should be discarded)
 *                                                       // intensity data [device-specific units].  If your
 *                                                       // device does not provide intensities, please leave the array empty.
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
#include <sensor_msgs/LaserScan.h>



class LaserTest
{
    /**
     * @brief Class to subscribe to the topic naoqi_driver/laser
     * 
     */

    public:
        LaserTest()
        {
            // Create a subscriber object to get laser sensor data
            sub = n.subscribe("/naoqi_driver/laser", 1, &LaserTest::laserCallBack, this);
        }

        void laserCallBack(const sensor_msgs::LaserScan& msg)
        {
            ROS_INFO_STREAM(" ------------ Printing laser sensor data -----------\n\n");

            ROS_INFO_STREAM("Frame id: " << msg.header.frame_id << "\n" );

            ROS_INFO_STREAM("Start angle of the scan: " << msg.angle_min << "\n" );

            ROS_INFO_STREAM("End angle of the scan: " << msg.angle_max << "\n" );

            ROS_INFO_STREAM("Angular distance between measurements: " << msg.angle_increment << "\n" );

            ROS_INFO_STREAM("Time between measurements: " << msg.time_increment << "\n" );

            ROS_INFO_STREAM("Time between scans: " << msg.scan_time << "\n" );

            ROS_INFO_STREAM("Minimum range value: " << msg.range_min << "\n" );

            ROS_INFO_STREAM("Maximum range value: " << msg.range_max << "\n" );

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
    ros::init(argc, argv, "laser_test");

    // instanciate an object from the class LaserTest
    LaserTest laserSensor; 

    // let ROS take over
    ros::spin(); 

    return 0;
}