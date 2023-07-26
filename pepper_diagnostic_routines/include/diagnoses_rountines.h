# include <ros/ros.h>
# include <ros/package.h>
# include <std_msgs/Float64.h>
# include <sensor_msgs/JointState.h>
# include <sensor_msgs/CameraInfo.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>
#include <iomanip>

using namespace std;


/* Call back functions executed when a sensor data arrived */

void jointStateCallBack(const sensor_msgs::JointState& state); 
void imageCallBack(const sensor_msgs::ImageConstPtr& msg);



