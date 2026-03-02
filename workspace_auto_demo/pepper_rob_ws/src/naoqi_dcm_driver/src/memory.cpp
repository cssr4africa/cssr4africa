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

#include "naoqi_dcm_driver/memory.hpp"
#include "naoqi_dcm_driver/tools.hpp"

Memory::Memory(const qi::SessionPtr& session)
{
  try
  {
    memory_proxy_ = session->service("ALMemory");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Memory: Failed to connect to Memory Proxy!\n\tTrace: %s", e.what());
  }
}

void Memory::init(const std::vector <std::string> &joints_names)
{
  keys_positions_ = initMemoryKeys(joints_names);
}

std::vector <std::string> Memory::initMemoryKeys(const std::vector <std::string> &joints)
{
  std::vector <std::string> keys;
  for(std::vector<std::string>::const_iterator it=joints.begin(); it!=joints.end(); ++it)
  {
    keys.push_back("Device/SubDeviceList/" + *it + "/Position/Sensor/Value");
  }
  return keys;
}

std::vector<float> Memory::getListData()
{
  return getListData(keys_positions_);
}

std::vector<float> Memory::getListData(const std::vector <std::string> &keys)
{
  qi::AnyValue keys_qi;

  try
  {
    keys_qi = memory_proxy_.call<qi::AnyValue>("getListData", keys);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Memory: Could not read joints data from Memory Proxy \n\tTrace: %s", e.what());
  }

  std::vector<float> joint_positions = fromAnyValueToFloatVector(keys_qi);
  return joint_positions;
}

std::string Memory::getData(const std::string &str)
{
  std::string res;
  try
  {
    res = memory_proxy_.call<std::string>("getData", str);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Memory: Failed to get the robot's name \n\tTrace: %s", e.what());
  }
  return res;
}

void Memory::subscribeToMicroEvent(const std::string &name,
                                   const std::string &callback_module,
                                   const std::string &callback_method,
                                   const std::string &callback_message)
{
  try
  {
    memory_proxy_.call<int>("subscribeToMicroEvent", name, callback_module, callback_message, callback_method);
  }
  catch(const std::exception& e)
  {
    ROS_WARN("Memory: Failed to subscribe to micro-event '%s'.\n\tTrace: %s", name.c_str(), e.what());
  }
}

void Memory::unsubscribeFromMicroEvent(const std::string &name,
                                       const std::string &callback_module)
{
  try
  {
    memory_proxy_.call<void>("unsubscribeToMicroEvent", name, callback_module);
  }
  catch(const std::exception& e)
  {
    ROS_WARN("Memory: Failed to unsubscribe from micro-event '%s'.\n\tTrace: %s", name.c_str(), e.what());
  }
}
