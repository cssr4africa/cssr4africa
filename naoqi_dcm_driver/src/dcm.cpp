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

#include "naoqi_dcm_driver/dcm.hpp"
#include "naoqi_dcm_driver/tools.hpp"

DCM::DCM(const qi::SessionPtr& session,
         const double &controller_freq):
  controller_freq_(controller_freq)
{
  try
  {
    dcm_proxy_ = session->service("DCM");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to connect to DCM Proxy!\n\tTrace: %s", e.what());
  }
}

bool DCM::init(const std::vector <std::string> &joints)
{
  // DCM Motion Commands Initialization
  createPositionActuatorCommand(joints);

  // Create an alias for Joints Actuators
  if (!createPositionActuatorAlias(joints))
    return false;

  // Create an alias for Joints Hardness
  if (!createHardnessActuatorAlias(joints))
    return false;

  return true;
}

void DCM::createPositionActuatorCommand(const std::vector <std::string> &joints)
{
  //get the number of active joints
  int joints_nbr = joints.size();

  // Create the Motion Command
  commands_.reserve(4);
  commands_.resize(4);
  commands_[0] = qi::AnyValue(qi::AnyReference::from("jointActuator"), false, false);
  commands_[1] = qi::AnyValue(qi::AnyReference::from("ClearAll"), false, false);
  commands_[2] = qi::AnyValue(qi::AnyReference::from("time-mixed"), false, false);

  // set keys
  commands_values_.reserve(joints_nbr);
  commands_values_.resize(joints_nbr);
  for(int i=0; i<joints_nbr; ++i)
  {
    commands_values_[i].resize(1);
    commands_values_[i][0].resize(3);
    commands_values_[i][0][2] = qi::AnyValue(qi::AnyReference::from(0), false, false);
  }

  commands_[3] = qi::AnyValue(qi::AnyReference::from(commands_values_), false, false);
}

bool DCM::createPositionActuatorAlias(const std::vector <std::string> &joints)
{
  // prepare the command
  std::vector <qi::AnyValue> commandAlias;
  commandAlias.reserve(2);
  commandAlias.resize(2);
  commandAlias[0] = qi::AnyValue(qi::AnyReference::from("jointActuator"), false, false);

  // set joints actuators keys
  std::vector <qi::AnyValue> commandAlias_keys;
  commandAlias_keys.resize(joints.size());
  for(int i=0; i<joints.size(); ++i)
  {
    std::string key = "Device/SubDeviceList/" + joints.at(i) + "/Position/Actuator/Value";
    commandAlias_keys[i] = qi::AnyValue(qi::AnyReference::from(key), false, false);
  }
  commandAlias[1] = qi::AnyValue(qi::AnyReference::from(commandAlias_keys), false, false);

  qi::AnyValue commandAlias_qi(qi::AnyReference::from(commandAlias), false, false);

  // Create alias
  try
  {
    dcm_proxy_.call<void>("createAlias", commandAlias_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Could not create alias for jonts positions!\n\tTrace: %s", e.what());
    return false;
  }
  return true;
}

bool DCM::createHardnessActuatorAlias(const std::vector <std::string> &joints)
{
  //prepare a command
  std::vector <qi::AnyValue> commandAlias;
  commandAlias.reserve(2);
  commandAlias.resize(2);
  commandAlias[0] = qi::AnyValue(qi::AnyReference::from("jointStiffness"), false, false);

  //set stiffness keys
  std::vector <qi::AnyValue> commandAlias_keys;
  for(int i=0; i<joints.size(); ++i)
  {
    if((joints.at(i) == "RHipYawPitch") //for mimic joints: Nao only
        || (joints.at(i).find("Wheel") != std::string::npos))
      continue;

    std::string key = "Device/SubDeviceList/" + joints.at(i) + "/Hardness/Actuator/Value";
    commandAlias_keys.push_back(qi::AnyValue(qi::AnyReference::from(key), false, false));
  }
  commandAlias[1] = qi::AnyValue(qi::AnyReference::from(commandAlias_keys), false, false);

  qi::AnyValue commandAlias_qi(qi::AnyReference::from(commandAlias), false, false);

  // Create alias
  try
  {
    dcm_proxy_.call<void>("createAlias", commandAlias_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Could not create alias for joints hardness!\n\tTrace: %s", e.what());
    return false;
  }
  return true;
}

bool DCM::DCMAliasTimedCommand(const std::string& alias,
                                 const float& value,
                                 const int& timeOffset,
                                 const std::string& type_update)
{
  //get DCM time at timeOffset
  int time = getTime(timeOffset);

  // Create Alias timed-command
  qi::AnyValue command_qi;
  try
  {
    std::vector <qi::AnyValue> command;
    command.resize(3);
    command[0] = qi::AnyValue(qi::AnyReference::from(alias), false, false);
    command[1] = qi::AnyValue(qi::AnyReference::from(type_update), false, false);

    std::vector <std::vector <qi::AnyValue> > command_keys;
    command_keys.resize(1);
    command_keys[0].resize(2);
    command_keys[0][0] = qi::AnyValue(qi::AnyReference::from(value), false, false);
    command_keys[0][1] = qi::AnyValue(qi::AnyReference::from(time), false, false);

    command[2] = qi::AnyValue(qi::AnyReference::from(command_keys), false, false);
    command_qi = qi::AnyValue(qi::AnyReference::from(command), false, false);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to convert to qi::AnyValue \n\tTrace: %s", e.what());
  }

  // Execute Alias timed-command
  try
  {
    dcm_proxy_.call<void>("set", command_qi);
    return false;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to execute DCM timed-command! \n\tTrace: %s", e.what());
  }
  return true;
}

int DCM::getTime(const int &offset)
{
  int res;
  try
  {
    res = dcm_proxy_.call<int>("getTime", 0) + offset;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to get time! \n\tTrace: %s", e.what());
  }
  return res;
}

void DCM::writeJoints(const std::vector <double> &joint_commands)
{
  int time = getTime(static_cast<int>(5000.0/controller_freq_));

  // Create Alias timed-command
  qi::AnyValue commands_qi;
  try
  {
    std::vector<double>::const_iterator it_comm = joint_commands.begin();
    for(int i=0; i<joint_commands.size(); ++i, ++it_comm)
    {
      commands_values_[i][0][0] = qi::AnyValue(qi::AnyReference::from(static_cast<float>(*it_comm)), false, false);
      commands_values_[i][0][1] = qi::AnyValue(qi::AnyReference::from(time), false, false);
    }

    commands_[3] = qi::AnyValue(qi::AnyReference::from(commands_values_), false, false);
    commands_qi = qi::AnyValue(qi::AnyReference::from(commands_), false, false);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to convert to qi::AnyValue \n\tTrace: %s", e.what());
  }

  // Execute Alias timed-command
  try
  {
    dcm_proxy_.call<void>("setAlias", commands_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DCM: Failed to execute DCM timed-command! \n\tTrace: %s", e.what());
  }
}

bool DCM::setStiffness(const float &stiffness)
{
  //set stiffness with 1sec timeOffset
  return DCMAliasTimedCommand("jointStiffness", stiffness, 1000);
}
