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

#ifndef TOOLS_HPP
#define TOOLS_HPP

// NAOqi Headers
#include <qi/anyvalue.hpp>

#include <ros/ros.h>

qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector);

qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector);

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value);

std::vector<int> fromAnyValueToIntVector(qi::AnyValue& value);

std::string print(const std::vector <std::string> &vector);

std::vector <std::string> toVector(const std::string &input);

void xmlToVector(XmlRpc::XmlRpcValue &topicList,
                std::vector <std::string> *joints);

#endif // TOOLS_HPP
