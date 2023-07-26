#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;

// global variables of the current pose Pepper
double current_x; 
double current_y;
double current_theta; 

/*
*  This call back function allows us to get Pepper's current position thanks to the /odom topic. 
*/
void odomReceived(const nav_msgs::Odometry::ConstPtr& msg)
{

    // get pose 
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    // we use the tf library to get the orientation of the robot
    // we gets the orientation as a quaternion thanks to tf2

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    ); 

    // we convert the quaternion to a (3,3) matrix 
    tf2::Matrix3x3 mat(q);

    // Euler angles 
    double roll, pitch, yaw;

    // we get the Euler angles from the matrix 
    mat.getRPY(roll, pitch, yaw);

    // the orientation theta of Pepper is the yaw 
    current_theta = yaw;
    
}

int main(int argc, char** argv)
{

    geometry_msgs::Twist msg;

    double start_x        = 2.0; 
    double start_y        = 1.0;
    double start_theta    = 3.14;

    double goal_x         = 8.0;
    double goal_y         = 9.0; 
    double goal_theta     = 0.0; 

    double goal_direction;

    double position_error; 
    double angle_error; 

    double delta_pos      = 0.5; 
    double delta_theta    = 0.05; 

    double kp_pos1        = 0.5;
    double kp_theta1      = 1.0;

    double publish_rate = 50;

    // Initialize the ROS system 
    ros::init(argc, argv, "goToPosition");
    
    // Create a node
    ros::NodeHandle nh;

    // Create a subscriber object to get the robot pose
    ros::Subscriber sub = nh.subscribe("/naoqi_driver/odom", 1000, &odomReceived);

    // Create a publisher object to publish velocity commands
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Publish at this rate (in Hz) until the node is shutdown  
    ros::Rate rate(publish_rate); 

    // start position of the robot
    current_x             = start_x;
    current_y             = start_y;
    current_theta         = start_theta; 

    // divide and conquer algorithm
    do
    {
        // Let ROS take over to handle the callback
        ros::spinOnce(); 

        position_error = sqrt((goal_x - current_x) * (goal_x - current_x) + 
                                  (goal_y - current_y) * (goal_y - current_y));

        goal_direction = atan2((goal_y - current_y), (goal_x - current_x));

        angle_error    = goal_direction - current_theta; 

        if(fabs(angle_error) > delta_theta)
        {
            msg.linear.x   = 0;
            msg.angular.z  = kp_theta1 * angle_error;
        } 
        else
        {
            msg.linear.x   = kp_pos1 * position_error;
            msg.angular.z  = 0;
        }

        if(position_error < delta_pos)
        {
            angle_error    = goal_theta - current_theta;
            msg.linear.x   = 0; 
            msg.angular.z  = kp_theta1 * angle_error;
        }

        pub.publish(msg);
        ROS_INFO_STREAM("Velocity command: " << " linear ="  << msg.linear.x
                                                 << " angular =" << msg.angular.z);

        rate.sleep();

    } while ((position_error >= delta_pos) && ros::ok());


    return 0;
}