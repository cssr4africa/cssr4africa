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

#include <diagnostic_msgs/DiagnosticArray.h>

#include "naoqi_dcm_driver/diagnostics.hpp"
#include "naoqi_dcm_driver/tools.hpp"

Diagnostics::Diagnostics(const qi::SessionPtr& session,
                         ros::Publisher *pub,
                         const std::vector<std::string> &joints_all_names,
                         const std::string &robot):
    pub_(pub),
    joints_all_names_(joints_all_names),
    temperature_warn_level_(68.0f),
    temperature_error_level_(73.0f)
{
  //resize the joint current vector
  joints_current_.reserve(joints_all_names_.size());
  joints_current_.resize(joints_all_names_.size());

  //set the default status
  status_.name = std::string("naoqi_dcm_driver:Status");
  status_.hardware_id = "robot";
  status_.level = diagnostic_msgs::DiagnosticStatus::OK;
  status_.message = "OK";

  //connect to Memmory proxy
  try
  {
    memory_proxy_ = session->service("ALMemory");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("DIAGNOSTICS: Failed to connect to Memory Proxy!\n\tTrace: %s", e.what());
  }

  //set the keys to check
  keys_tocheck_.push_back("Device/SubDeviceList/Battery/Charge/Sensor/Value");

  std::vector<std::string>::const_iterator it = joints_all_names_.begin();
  for(; it != joints_all_names_.end(); ++it) {
    keys_tocheck_.push_back("Device/SubDeviceList/" + *it + "/Temperature/Sensor/Value");
    keys_tocheck_.push_back("Device/SubDeviceList/" + *it + "/Hardness/Actuator/Value");
    keys_tocheck_.push_back("Device/SubDeviceList/" + *it + "/ElectricCurrent/Sensor/Value");
  }

  std::vector<std::string>::const_iterator it_cntrl = joints_all_names_.begin();
  for(; it_cntrl != joints_all_names_.end(); ++it_cntrl)
    keys_tocheck_.push_back("Device/SubDeviceList/" + *it_cntrl + "/ElectricCurrent/Sensor/Value");

  //allow the temperature reporting (for CPU)
  try
  {
    if ((robot == "pepper") || (robot == "juliette") || (robot == "nao")) {
      qi::AnyObject body_temperature_ = session->service("ALBodyTemperature");
      body_temperature_.call<void>("setEnableNotifications", true);
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("DIAGNOSTICS: Failed to connect to ALBodyTemperature!\n\tTrace: %s", e.what());
  }
}

void Diagnostics::setMessageFromStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
{
  if (status.level == diagnostic_msgs::DiagnosticStatus::OK) {
    status.message = "OK";
  } else if (status.level == diagnostic_msgs::DiagnosticStatus::WARN) {
    status.message = "WARN";
  } else {
    status.message = "ERROR";
   }
 }

void Diagnostics::setAggregatedMessage(diagnostic_updater::DiagnosticStatusWrapper &status)
{
  if(status.level > status_.level) {
    status_.level = status.level;
    status_.message = status.message;
  }
}

bool Diagnostics::publish()
{
  diagnostic_msgs::DiagnosticArray msg;
  msg.header.stamp = ros::Time::now();

  //set the default status
  status_.level = diagnostic_msgs::DiagnosticStatus::OK;
  status_.message = "OK";

  // Fill the temperature / stiffness message for the joints
  float maxTemperature = 0.0f;
  float maxStiffness = 0.0f;
  float minStiffness = 1.0f;
  float minStiffnessWoHands = 1.0f;
  float maxCurrent = 0.0f;
  float minCurrent = 10.0f;
  std::stringstream hotJointsSS;
  diagnostic_msgs::DiagnosticStatus::_level_type max_level = diagnostic_msgs::DiagnosticStatus::OK;

  std::vector<float> values;
  try
  {
    qi::AnyValue keys_tocheck_qi = memory_proxy_.call<qi::AnyValue>("getListData", keys_tocheck_);
    values = fromAnyValueToFloatVector(keys_tocheck_qi);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("DIAGNOSTICS: Could not get joint data from the robot \n\tTrace: %s", e.what());
    return false;
  }

  //check the battery charge level
  size_t val = 0;
  float batteryCharge = static_cast<float>(values[val++]);

  diagnostic_updater::DiagnosticStatusWrapper status_battery;
  status_battery.name = std::string("naoqi_dcm_driver:Battery");
  status_battery.hardware_id = "battery";
  status_battery.add("BatteryCharge", batteryCharge);

  //TODO check if it is charging
  if (batteryCharge > 5.0f)
  {
    status_battery.level = diagnostic_msgs::DiagnosticStatus::OK;
    status_battery.message = "OK";
  }
  else
  {
    status_battery.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status_battery.message = "LOW Battery Charge";
  }
  msg.status.push_back(status_battery);

  std::vector<std::string>::iterator it_name = joints_all_names_.begin();
  std::vector<float>::iterator it_current = joints_current_.begin();
  for(; it_name != joints_all_names_.end(); ++it_name, ++it_current)
  {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = std::string("naoqi_dcm_driver:") + *it_name;

    float temperature = static_cast<float>(values[val++]);
    float stiffness = static_cast<float>(values[val++]);
    *it_current = static_cast<float>(values[val++]);

    // Fill the status data
    status.hardware_id = *it_name;
    status.add("Temperature", temperature);
    status.add("Stiffness", stiffness);
    status.add("ElectricCurrent", *it_current);

    // Define the level
    if (temperature < temperature_warn_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "OK";
    }
    else if (temperature < temperature_error_level_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Hot";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "HIGH JOINT TEMPERATURE : " + *it_name;
    }

    msg.status.push_back(status);
    setAggregatedMessage(status);

    // Fill the joint data for later processing
    max_level = std::max(max_level, status.level);
    maxTemperature = std::max(maxTemperature, temperature);
    maxStiffness = std::max(maxStiffness, stiffness);
    minStiffness = std::min(minStiffness, stiffness);
    if((*it_name).find("Hand") == std::string::npos)
      minStiffnessWoHands = std::min(minStiffnessWoHands, stiffness);
    maxCurrent = std::max(maxCurrent, *it_current);
    minCurrent = std::min(minCurrent, *it_current);
    if(status.level >= (int) diagnostic_msgs::DiagnosticStatus::WARN) {
      hotJointsSS << std::endl << *it_name << ": " << temperature << "°C";
    }
  }

  // Get the aggregated joints status
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.name = std::string("naoqi_dcm_driver:Status");
  status.hardware_id = "joints";
  status.level = max_level;
  setMessageFromStatus(status);

  status.add("Highest Temperature", maxTemperature);
  status.add("Highest Stiffness", maxStiffness);
  status.add("Lowest Stiffness", minStiffness);
  status.add("Lowest Stiffness without Hands", minStiffnessWoHands);
  status.add("Highest Electric Current", maxCurrent);
  status.add("Lowest Electric current", minCurrent);
  status.add("Hot Joints", hotJointsSS.str());

  msg.status.push_back(status);

  pub_->publish(msg);

  if(status_.level >= (int) diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    ROS_ERROR_STREAM("DIAGNOSTICS: ERROR DETECTED: " << getStatusMsg());
    return false;
  }
  else
    return true;
}

std::string Diagnostics::getStatusMsg()
{
  return status_.message;
}
