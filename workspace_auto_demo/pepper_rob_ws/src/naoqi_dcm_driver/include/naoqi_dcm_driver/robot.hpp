/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef NAOQI_DCM_DRIVER_H
#define NAOQI_DCM_DRIVER_H

// Boost Headers
#include <boost/shared_ptr.hpp>

// NAOqi Headers
#include <qi/session.hpp>
#include <qi/anyobject.hpp>
#include <qi/os.hpp>
#include <qi/anyvalue.hpp>

// ROS Headers
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include "naoqi_dcm_driver/diagnostics.hpp"
#include "naoqi_dcm_driver/memory.hpp"
#include "naoqi_dcm_driver/dcm.hpp"
#include "naoqi_dcm_driver/motion.hpp"

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

class Robot : public hardware_interface::RobotHW
{
public:
  /**
  * @brief Constructor
  * @param session[in] session pointer for the service registration
  */
  Robot(qi::SessionPtr session);

  //! @brief destroy all ros nodehandle and shutsdown all publisher
  ~Robot();

  //! @brief stop the service
  void stopService();

  //! @brief check if the service is connected
  bool isConnected();

  //! @brief connect to ALProxies
  bool connect();

  //! @brief start the main loop
  void run();

private:
  //! @brief initialize controllers based on joints names
  bool initializeControllers(const std::vector <std::string> &joints_names);

  //! @brief define Subscribe/Advertise to ROS Topics/Services
  void subscribe();

  //! @brief load parameters
  bool loadParams();

  //! @brief the main loop
  void controllerLoop();

  //! @brief control the robot's velocity
  void commandVelocity(const geometry_msgs::TwistConstPtr &msg);

  //! @brief publish the base_footprint
  void publishBaseFootprint(const ros::Time &ts);

  //! @brief check HW and Naoqi joints names
  std::vector <bool> checkJoints();

  //! @brief read joints values
  void readJoints();

  //! @brief publish joint states
  void publishJointStateFromAlMotion();

  //! @brief set joints values
  void writeJoints();

  //! @brief set stiffness
  bool setStiffness(const float &stiffness);

  //! @brief ignore mimic joints from control
  void ignoreMimicJoints(std::vector <std::string> *joints);

  /** node handle pointer*/
  boost::scoped_ptr <ros::NodeHandle> nhPtr_;

  /** pointer to Diagnostics class */
  boost::shared_ptr <Diagnostics> diagnostics_;

  /** pointer to Memory class */
  boost::shared_ptr <Memory> memory_;

  /** pointer to DCM class */
  boost::shared_ptr <DCM> dcm_;

  /** pointer to Motion class */
  boost::shared_ptr <Motion> motion_;

  /** subscrier to MoveTo */
  ros::Subscriber cmd_moveto_sub_;

  /** base_footprint broadcaster */
  tf::TransformBroadcaster base_footprint_broadcaster_;

  /** base_footprint listener */
  tf::TransformListener base_footprint_listener_;

  /** stiffness publisher */
  ros::Publisher stiffness_pub_;

  /** diagnostics publisher */
  ros::Publisher diag_pub_;

  /** joint states publisher */
  ros::Publisher joint_states_pub_;

  /** stiffness data */
  std_msgs::Float32 stiffness_;

  /** joint states data */
  sensor_msgs::JointState joint_states_topic_;

  /** controller manager */
  controller_manager::ControllerManager* manager_;

  /** service name */
  std::string session_name_;

  /** session connection status */
  bool is_connected_;

  /** robot body type */
  std::string body_type_;

  /** prefix for published topics */
  std::string prefix_;

  /** odom frame name */
  std::string odom_frame_;

  /** message buffer */
  int topic_queue_;

  /** frequency to read joints values */
  double high_freq_;

  /** frequency to write joints values */
  double controller_freq_;

  /** threshold to update joints to desired values */
  double joint_precision_;

  /** Naoqi session pointer */
  qi::SessionPtr _session;

  /** motor groups used to control */
  std::vector <std::string> motor_groups_;

  /** joints states from ROS hardware interface */
  hardware_interface::JointStateInterface jnt_state_interface_;

  /** joints positions from ROS hardware interface */
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  /** Naoqi joints names */
  std::vector <std::string> qi_joints_;

  /** Naoqi joints angles to apply */
  std::vector <double> qi_commands_;

  /** hardware interface joints names */
  std::vector <std::string> hw_joints_;

  /** hardware interface enabled joints */
  std::vector <bool> hw_enabled_;

  /** hardware interface joints angles to apply */
  std::vector <double> hw_commands_;

  /** hardware interface current joints angles */
  std::vector <double> hw_angles_;

  /** hardware interface joints velocities */
  std::vector <double> hw_velocities_;

  /** hardware interface joints efforts*/
  std::vector <double> hw_efforts_;

  /** enable using DCM instead of ALMotion */
  bool use_dcm_;

  /** stiffness value to apply */
  float stiffness_value_;
};

#endif // NAOQI_DCM_DRIVER_H
