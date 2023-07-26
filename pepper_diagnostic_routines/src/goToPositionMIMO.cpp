/********************************************************************************************************************************************
 *  Pepper diagnostic routines: Implementation of MIMO algorithm go to position 
 *  @file goToPositionMIMO.cpp
 *  @details The program  implements MIMO, a localization algorithm to make the robot move from a point A to a point B.
 *           The end goal is to test how good is the robot odometry. The tf library has been used to find the robot odometry. 
 *           When the robot's current pose is different from the goal pose we use the MIMO algorithm to reorient the robot toward the 
 *           goal pose. The goal and start poses are set as global variables in the current version of the code. 
 *            
 *  @author Pamely ZANTOU. This program has been adapted from Prof David Vernon implementation of the MIMO algorithm
 *          Source: Robotics Principle and Practice - Assignment 5
 *  @version 1.0 
 *  @date July 2022
 * 
 *  Audit Trail
 *  -----------
 * 
 * 
 * ********************************************************************************************************************************************/



#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <cmath>

using namespace std;

// macro definitions

#define PUBLISH_RATE 50

// locomotion parameter data

typedef struct 
{
    double position_tolerance;
    double angle_tolerance_orienting;
    double angle_tolerance_going;
    double position_gain_mimo; 
    double angle_gain_mimo; 
    double min_linear_velocity;
    double max_linear_velocity;
    double min_angular_velocity;
    double max_angular_velocity;
    
} locomotionParameterDataType;



// global variables of the current pose Pepper
double current_x; 
double current_y;
double current_theta; 



class GoToPositionMIMO
{
    /**
     * @brief Go to position using MIMO and tf
     * 
     */

    public:
        GoToPositionMIMO()
        {
            // set locomotion data parameters
            this->locomotionParameterData->position_tolerance             = 0.025;
            this->locomotionParameterData->angle_tolerance_orienting      = 0.075;
            this->locomotionParameterData->angle_tolerance_going          = 0.075;
            this->locomotionParameterData->position_gain_mimo             = 0.2;
            this->locomotionParameterData->angle_gain_mimo                = 0.5;
            this->locomotionParameterData->min_linear_velocity            = 0.015;
            this->locomotionParameterData->max_linear_velocity            = 0.5;
            this->locomotionParameterData->min_angular_velocity           = 0.09;
            this->locomotionParameterData->max_angular_velocity           = 1.0;

            // Create a subscriber object to get the robot pose


        }

        GoToPositionMIMO(start_x, start_y, start_z, goal_x, goal_y, goal_theta, delta_pos, delta_theta, kp_pos, kp_theta, current_linear_velocity)
        {
            this->start_x                           = start_x;
            this->start_y                           = start_y;
            this->start_theta                       = start_z;

            this->goal_x                            = goal_x;
            this->goal_y                            = goal_y;
            this->goal_theta                        = goal_theta;

            this->delta_pos                         = delta_pos;
            this->delta_theta                       = delta_theta;

            this->kp_pos                            = kp_pos;
            this->kp_theta                          = kp_theta;

            this->current_linear_velocity           = current_linear_velocity;
        }

        /* This call back function allows us to get Pepper's current position thanks to the /odom topic. */
        void odomMessageReceived(const nav_msgs::Odometry::ConstPtr& msg)
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

        // Setter --------------------------------------------------------------------------

        // set start pose of the robot
        void setStartPose(double x, double y, double theta)
        {
            this->start_x       = x;
            this->start_y       = y;
            this->start_theta   = theta;
        }

        // set goal pose
        void setGoalPose(double x, double y, double theta)
        {
            this->goal_x        = x;
            this->goal_y        = y;
            this->goal_theta    = theta;
        }

        // set proportional position gain
        void setPositionGain(double kp)
        {
            this->kp_pos        = kp;
        }

        // set proportional angular gain 
        void setAngularGain(double kp)
        {
            this->kp_theta      = kp;
        }

        // set treshold delta
        void setDelta(double pos, double theta)
        {
            this->delta_pos     = pos;
            this->delta_theta   = theta;
        }

        // set position error
        void setPositionError(double pos_error)
        {
            this->position_error = pos_error;
        }

        // set angle_error
        void setAngleError(double angl_error)
        {
            this->angle_error    = angl_error;
        }

        // set linear velocity 
        void setLinearVelocity(double lin_vel)
        {
            this->linear_velocity = lin_vel;
        }

        // set angular velocity
        void setAngularVelocity(double angl_vel)
        {
            this->angular_velocity = angl_vel;
        }

        // Getter ------------------------------------------------------------------------------------

        // Get start pose  

        tuple<double, double, double> getStartPose()
        {
            // we will return a tuple object of the robot start pose: (start_x, start_y, start_theta)

            tuple<double, double, double> start_pose; 

            start_pose =  make_tuple(this->start_x, this->start_y, this->start_theta );

            return start_pose;
        }

        // Get goal pose 

        tuple <double, double, double> getGoalPose()
        {
            // we will return a tuple object of the robot goal pose: (goal_x, goal_y, goal_theta)

            tuple<double, double, double> goal_pose;

            goal_pose = make_tuple(this->goal_x, this->goal_y, this->goal_theta);

            return goal_pose;
        }

        // Get  proportional position gain 
        double getPositionGain()
        {
            return this->kp_pos;
        }

        // Get  proportional angular gain 
        double getAngularGain()
        {
            return this->kp_theta;
        }

        // Get threshold double pos, double theta
        tuple<double, double>getDelta()
        {
            // we will return a tuple object: (delta_pos, delta_theta)

            delta = make_tuple(this->delta_pos, this->delta_theta);

            return delta;
        }

        // Get position error 
        
        double getPositionError()
        {
            return this->position_error;
        }

        // getter angle error
        double getAngleError()
        {
            return this->angle_error;
        }


        double getLinearVelocity()
        {
            return this->linear_error;
        }

        double getAngularVelocity()
        {
            return  this->angular_velocity;
        }

        /* return the sign of a number as +/- 1 */
        int signnum(float x)
        {
            if(x >= 0.0) return 1;
            if(x < 0.0) return -1;
        }


        void goToMimo(double x, double y, double theta, locomotionParameterDataType *locomotionParameterData, ros::Publisher pub, ros::Rate rate)
        {
            do
            {
                /* get current pose */

                ros::spinOnce();  // let ROS take over

                this->position_error  = sqrt((this->goal_x - current_x) * (this->goal_x - current_x) + 
                                            (this->goal_y - current_y) * (this->goal_y - current_y)); 

                this->goal_direction = atan2((this->goal_y - current_y), (this->goal_x - current_x));

                this->angle_error = this->goal_direction - current_theta; 

                if (this->angle_error > M_PI)
                {
                    this->angle_error = angle_error - 2 * M_PI;
                }
                else if (this->angle_error < -M_PI)
                {
                    angle_error = angle_error + 2 * M_PI;
                }

                this->current_linear_velocity = 0;
                this->msg.linear.x            = current_linear_velocity;

                this->angular_velocity = locomotionParameterData->angle_gain_mimo * angle_error;

                if(fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
                {
                    this->msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(this->angular_velocity);
                }
                else if(fabs(angular_velocity) > locomotionParameterData.max_angular_velocity) 
                {
                    this->msg.angular.z = locomotionParameterData.max_angular_velocity * signum(angular_velocity);
                }
                else
                {
                    this->msg.angular.z = angular_velocity;
                }
                
                printf("Error: %5.3f %5.3f\n", this->position_error, this->angle_error);
                printf("Velocity command: %5.3f %5.3f\n", this->msg.linear.x, this->msg.angular.z);

                pub.publish(this->msg);

                rate.sleep();


            } while (fabs(this->position_error) > locomotionParameterData->position_tolerance && ros::ok());
            
        }



    private:

        // create a node
        ros::NodeHandle n; 

        // Create a publisher object to publish velocity commands
        ros::Publiser   pub;

        // Create a subscriber object to get the robot pose
        ros::Subscriber sub;

        // Publish at this rate (in Hz) until the node is shutdown  
        ros::Rate rate(PUBLISH_RATE); 

        geometry_msgs::Twist msg;

        locomotionParameterDataType *locomotionParameterData;

        double start_x    
        double start_y;
        double start_theta;

        double goal_x;
        double goal_y; 
        double goal_theta; 

        double goal_direction;

        double position_error; 
        double angle_error; 

        double delta_pos; 
        double delta_theta; 

        double kp_pos;
        double kp_theta;

        double linear_velocity;
        double angular_velocity;

        static double current_linear_velocity; // make it static so that it is still valid on the next call 
        

};


int main(int argc, char** argv)
{

    return 0;
}