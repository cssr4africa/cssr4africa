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

#ifndef MOTION_HPP
#define MOTION_HPP

// NAOqi Headers
#include <qi/session.hpp>

/**
 * @brief This class is a wapper for Naoqi Motion Class
 */
class Motion
{
public:
  Motion(const qi::SessionPtr& session);

  //! @brief initialize with joints names to control
  void init(const std::vector <std::string> &joints_names);

  //! @brief check if the robot is waked up
  bool robotIsWakeUp();

  //! @brief wake up the robot
  void wakeUp();

  //! @brief go to the rest position
  void rest();

  //! @brief get body names
  std::vector <std::string> getBodyNames(const std::string &robot_part);

  //! @brief get body names based on motor groups
  std::vector <std::string> getBodyNamesFromGroup(const std::vector<std::string> &motor_groups);

  //! @brief Manage concurrence of DCM and ALMotion
  void manageConcurrence();

  //! @brief Move the robot at given velocity and angle
  void moveTo(const float& vel_x, const float& vel_y, const float& vel_th);

  //! @brief get joints angles
  std::vector<double> getAngles(const std::string &robot_part);

  //! @brief set joints values
  void writeJoints(const std::vector <double> &joint_commands);

  //! @brief set stiffness for one motor group
  bool stiffnessInterpolation(const std::string &motor_group,
                              const float &stiffness,
                              const float &time);

  //! @brief set stiffness for motors groups
  bool stiffnessInterpolation(const std::vector<std::string> &motor_groups,
                              const float &stiffness,
                              const float &time);

  //! @brief set stiffness for arms
  bool setStiffnessArms(const float &stiffness, const float &time);

private:
  /** Motion proxy */
  qi::AnyObject motion_proxy_;

  /** joints names */
  std::vector <std::string> joints_names_;
};

#endif // MOTION_HPP
