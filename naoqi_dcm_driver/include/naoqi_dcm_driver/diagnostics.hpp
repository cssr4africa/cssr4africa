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

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

// NAOqi Headers
#include <qi/session.hpp>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

/**
 * @brief This class defines a Diagnostic
 * It is used to check the robot state and sent to requesting nodes
 */
class Diagnostics
{
public:
  /**
  * @brief Constructor
  * @param session[in] Naoqi session
  * @param pub[in] ROS topic publisher
  * @param joints_all_names[in] all joints to check
  * @param robot[in] robot type
  */
  Diagnostics(const qi::SessionPtr& session,
              ros::Publisher *pub,
              const std::vector<std::string> &joints_all_names,
              const std::string &robot);

  //! @brief destroys all ros nodehandle and shutsdown all publisher
  virtual ~Diagnostics() {}

  //! @brief publish the newly received data
  bool publish();

  //! @brief set the message based on level
  void setMessageFromStatus(diagnostic_updater::DiagnosticStatusWrapper &status);

  //! @brief set the aggregated message
  void setAggregatedMessage(diagnostic_updater::DiagnosticStatusWrapper &status);

  //! @brief return the status message
  std::string getStatusMsg();

private:
  /** diagnostics publisher */
  ros::Publisher *pub_;

  /** Memory proxy */
  qi::AnyObject memory_proxy_;

  /** joints names */
  std::vector <std::string> joints_all_names_;

  /** joints electric current */
  std::vector <float> joints_current_;

  /** all the keys to check. It is a concatenation of
   * temperatures_keys, stiffness_keys, current_keys */
  std::vector <std::string> keys_tocheck_;

  /** the temperature to alert a warning */
  float temperature_warn_level_;

  /** the temperature to alert an error */
  float temperature_error_level_;

  /** The status message */
  diagnostic_updater::DiagnosticStatusWrapper status_;
};

#endif // DIAGNOSTICS_H
