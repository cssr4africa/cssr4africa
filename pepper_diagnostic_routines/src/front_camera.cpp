/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Range reading with Pepper's laser sensor
 *  @file front_camera.cpp
 *  @details The program subscribes to the topic /naoqi_driver/camera/front/image_raw which publishes 
 *           the front camera data. We subscribes to /naoqi_driver/camera/front/image_raw through to
 *           an image_transport object. This object is an image handler that transports raw image. After 
 *           getting the image, we convert it from ROS sensor_msgs/Image format to OpenCv cv::Mat format 
 *           with the cv_bridge package. We display an BRG images from OpenCv. 
 *  
 *  @author Pamely ZANTOU
 *  @version 1.0 
 *  @date August 2022
 * 
 *  Audit Trail
 *  -----------
 * 
 * 
 * ********************************************************************************************************************************************/

# include <ros/ros.h>
# include <ros/package.h>
# include <std_msgs/Float64.h>
# include <sensor_msgs/CameraInfo.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iomanip>



class FrontCameraTest
{
    /**
     * @brief Class to subscribe to the topic /naoqi_driver/camera/front/image_raw
     * 
     */

    public:
        FrontCameraTest(ros::NodeHandle &nh, image_transport::ImageTransport &it)
        {
            // Create a subscriber object to get camera raw image
            im_sub = it.subscribe("/naoqi_driver/camera/front/image_raw", 1, &FrontCameraTest::imageCallBack, this);
        }

        void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
        {
            // create an image pointer
            cv_bridge::CvImagePtr cv_ptr;

            try
            {
                {
                    //  convert to BGR image
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                }
            }
            catch(const cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat depth_float_img = cv_ptr->image;

            cv::imshow("view", depth_float_img);

            cv::waitKey(30);
            
        }


    private:

        // create an image transport subscribet object
        image_transport::Subscriber im_sub;

        
       
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "front_camera");

    // Create a node
    ros::NodeHandle nh;

    // image transport node to publish and subscribe to images
    image_transport::ImageTransport it(nh); 

    cv::namedWindow("view",  cv::WINDOW_AUTOSIZE);

    // instanciate an object from the class FrontCameraTest

    FrontCameraTest front_camera(nh, it);

    ros::spin();
    
    cv::destroyWindow("view");
    
    return 0;
}