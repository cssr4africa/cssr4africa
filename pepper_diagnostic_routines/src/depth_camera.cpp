# include <ros/ros.h>
# include <ros/package.h>
# include <std_msgs/Float64.h>
# include <sensor_msgs/JointState.h>
# include <sensor_msgs/CameraInfo.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <iomanip>



void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        {

            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        }
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
    }

    //Copy the image.data to imageBuf.

    cv::Mat depth_float_img = cv_ptr->image;

    double min = 0;

    double max = 1000;

    cv::Mat img_scaled_8u;
    cv::Mat color_img;
    

    cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));

    if(img_scaled_8u.type() ==  CV_8UC1)
    {
      
       cv::cvtColor(img_scaled_8u, color_img, CV_GRAY2RGB); 
    }

    cv::imshow("view", color_img);

    cv::waitKey(30);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_diagnostic");

    ros::NodeHandle nh;

    cv::namedWindow("view",  cv::WINDOW_AUTOSIZE);

    image_transport::ImageTransport it (nh);

    image_transport::Subscriber im_sub = it.subscribe("/naoqi_driver/camera/depth/image_raw", 1, imageCallBack);

    ros::spin();
    
    cv::destroyWindow("view");
    
    return 0;
}