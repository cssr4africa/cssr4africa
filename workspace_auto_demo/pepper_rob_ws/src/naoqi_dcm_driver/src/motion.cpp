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

// ROS Headers
#include <ros/ros.h>

#include "naoqi_dcm_driver/motion.hpp"
#include "naoqi_dcm_driver/tools.hpp"

Motion::Motion(const qi::SessionPtr& session)
{
  try
  {
    motion_proxy_ = session->service("ALMotion");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Motion: Failed to connect to Motion Proxy!\n\tTrace: %s", e.what());
  }

  //set walk arms enabled / disabled
  try
  {
    motion_proxy_.call<void>("setMoveArmsEnabled", 0, 0);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to set move arms enabled!\n\tTrace: %s", e.what());
  }

  //set external collision protection of the robot
  try
  {
    motion_proxy_.call<void>("setExternalCollisionProtectionEnabled", "Arms", 0);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to set external collision protection!\n\tTrace: %s", e.what());
  }

  //set Smart Stiffness
  try
  {
    motion_proxy_.call<void>("setSmartStiffnessEnabled", 1);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to set smart stiffness!\n\tTrace: %s", e.what());
  }
}

void Motion::init(const std::vector <std::string> &joints_names)
{
  joints_names_ = joints_names;
}

bool Motion::robotIsWakeUp()
{
  bool res(false);
  try
  {
    if (motion_proxy_.call<bool>("robotIsWakeUp"))
      res = true;
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to check if the robot is wakedup!\n\tTrace: %s", e.what());
  }
  return res;
}

void Motion::wakeUp()
{
  try
  {
    if (!robotIsWakeUp())
    {
      ROS_INFO_STREAM("Going to wakeup ...");
      motion_proxy_.call<void>("wakeUp");
      ros::Duration(3.0).sleep();
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to wake up the robot!\n\tTrace: %s", e.what());
  }
}

void Motion::rest()
{
  try
  {
    if (motion_proxy_.call<bool>("robotIsWakeUp"))
    {
      ROS_INFO_STREAM("Going to rest ...");
      motion_proxy_.call<void>("rest");
      sleep(4);
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to go to rest!\n\tTrace: %s", e.what());
  }
}

std::vector <std::string> Motion::getBodyNames(const std::string &robot_part)
{
  std::vector <std::string> joints;
  try
  {
    joints = motion_proxy_.call< std::vector<std::string> >("getBodyNames", robot_part);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Motion: Failed to get robot body names!\n\tTrace: %s", e.what());
  }

  return joints;
}

std::vector <std::string> Motion::getBodyNamesFromGroup(const std::vector<std::string> &motor_groups)
{
  std::vector <std::string> res;

  for (std::vector<std::string>::const_iterator it=motor_groups.begin(); it!=motor_groups.end(); ++it)
  {
    std::vector <std::string> joint_names = getBodyNames(*it);
    res.insert(res.end(), joint_names.begin(), joint_names.end());
  }
  return res;
}

void Motion::manageConcurrence()
{
  //set Smart Stiffness
  try
  {
    motion_proxy_.call<void>("setSmartStiffnessEnabled", 0);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to set external collision protection!\n\tTrace: %s", e.what());
  }

  //set PushRecoveryEnabled
  try
  {
    motion_proxy_.call<void>("setPushRecoveryEnabled", 0);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to set Push Recovery Enabled!\n\tTrace: %s", e.what());
  }
}

void Motion::moveTo(const float& vel_x, const float& vel_y, const float& vel_th)
{
  ROS_INFO_STREAM("going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th);

  try
  {
    motion_proxy_.call<void>("moveTo", vel_x, vel_y, vel_th);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to execute MoveTo!\n\tTrace: %s", e.what());
  }
}

std::vector<double> Motion::getAngles(const std::string &robot_part)
{
  std::vector<double> res;

  try
  {
    res = motion_proxy_.call< std::vector<double> >("getAngles", robot_part, 1);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Motion: Failed to get joints angles!\n\tTrace: %s", e.what());
  }

  return res;
}

// void Motion::writeJoints(const std::vector <double> &joint_commands)
// {
//   //prepare the list of joints
//   qi::AnyValue names_qi = fromStringVectorToAnyValue(joints_names_);

//   //prepare the list of joint angles
//   qi::AnyValue angles_qi = fromDoubleVectorToAnyValue(joint_commands);

//   try
//   {
//     motion_proxy_.async<void>("setAngles", names_qi, angles_qi, 0.2f);
//   }
//   catch(const std::exception& e)
//   {
//     ROS_ERROR("Motion: Failed to set joints nagles! \n\tTrace: %s", e.what());
//   }
// }

void Motion::writeJoints(const std::vector <double> &joint_commands)
{
  try
  {
    motion_proxy_.async<void>("setAngles", joints_names_, joint_commands, 0.2f);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Motion: Failed to set joints angles! \n\tTrace: %s", e.what());
  }
}

bool Motion::stiffnessInterpolation(const std::vector<std::string> &motor_groups,
                                    const float &stiffness,
                                    const float &time)
{
  std::vector<std::string>::const_iterator it=motor_groups.begin();
  for (; it!=motor_groups.end(); ++it)
    if (!stiffnessInterpolation(*it, stiffness, time))
      return false;
  return true;
}

bool Motion::stiffnessInterpolation(const std::string &motor_group,
                                    const float &stiffness,
                                    const float &time)
{
  try
  {
    motion_proxy_.call<void>("stiffnessInterpolation", motor_group, stiffness, time);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Motion: Failed to set stiffness \n\tTrace: %s", e.what());
    return false;
  }

  ROS_INFO_STREAM("Stiffness is updated to " << stiffness << " for " << motor_group);

  return true;
}

bool Motion::setStiffnessArms(const float &stiffness, const float &time)
{
  if (!stiffnessInterpolation("LArm", stiffness, time))
    return false;
  if (!stiffnessInterpolation("RArm", stiffness, time))
    return false;

  return true;
}
