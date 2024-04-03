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

#include <boost/algorithm/string.hpp>

#include "naoqi_dcm_driver/tools.hpp"

qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<std::string>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(*it), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}

qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<double>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(static_cast<float>(*it)), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value)
{
  std::vector<float> result;
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size(); ++i)
  {
    try
    {
      result.push_back(anyrefs[i].content().toFloat());
    }
    catch(std::runtime_error& e)
    {
      result.push_back(0.0f);
      std::cout << e.what() << "=> set to 0.0f" << std::endl;
    }
  }
  return result;
}

std::vector<int> fromAnyValueToIntVector(qi::AnyValue& value)
{
  std::vector<int> result;
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size();i++)
  {
    try
    {
      result.push_back(anyrefs[i].content().toInt());
    }
    catch(std::runtime_error& e)
    {
      result.push_back(-1);
      std::cout << e.what() << "=> set to -1" << std::endl;
    }
  }
  return result;
}

std::string print(const std::vector <std::string> &vector)
{
  std::stringstream ss;
  if (vector.size() > 0)
  {
    std::copy(vector.begin(), vector.end()-1, std::ostream_iterator<std::string>(ss,", "));
    std::copy(vector.end()-1, vector.end(), std::ostream_iterator<std::string>(ss));
  }
  return ss.str();
}

std::vector <std::string> toVector(const std::string &input)
{
  std::vector <std::string> value;
  boost::split (value, input, boost::is_any_of(" "));

  //check for empty values
  if (!value.empty())
    for(std::vector<std::string>::iterator it=value.begin(); it != value.end(); ++it)
      if ((*it).empty())
      {
        value.erase(it);
        it--;
      }
  return value;
}

void xmlToVector(XmlRpc::XmlRpcValue &topicList,
                std::vector <std::string> *joints)
{
  if (topicList.size() == 0)
  {
    ROS_WARN("Mentioned controller does not have joints");
    return;
  }
  for (int i = 0; i < topicList.size(); ++i)
  {
    std::string tmp = static_cast<std::string>(topicList[i]);
    if (tmp.compare("") != std::string::npos)
      joints->push_back(tmp);
  }
}
