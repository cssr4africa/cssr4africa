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

// NAOqi Headers
#include <qi/application.hpp>

#include "naoqi_dcm_driver/robot.hpp"

static std::string getROSIP(std::string network_interface)
{
  if (network_interface.empty())
    network_interface = "eth0";

  typedef std::map< std::string, std::vector<std::string> > Map_IP;
  Map_IP map_ip = static_cast<Map_IP>(qi::os::hostIPAddrs());
  if ( map_ip.find(network_interface) == map_ip.end() ) {
    std::cerr << "Could not find network interface named " << network_interface << ", possible interfaces are ... ";
    for (Map_IP::iterator it=map_ip.begin(); it!=map_ip.end(); ++it) std::cerr << it->first <<  " ";
    std::cerr << std::endl;
    exit(1);
  }

  static const std::string ip = map_ip[network_interface][0];
  return ip;
}

static void setMasterURINet( const std::string& uri, const std::string& network_interface )
{
  setenv("ROS_MASTER_URI", uri.c_str(), 1);

  std::string my_master = "__master="+uri;
  std::map< std::string, std::string > remap;
  remap["__master"] = uri;
  remap["__ip"] = getROSIP(network_interface);
  // init ros without a sigint-handler in order to shutdown correctly by naoqi
  const char* ns_env = std::getenv("ROS_NAMESPACE");

  ros::init( remap, (ns_env==NULL)?(std::string("naoqi_dcm_driver")):("") , ros::init_options::NoSigintHandler );
}

int main(int argc, char** argv)
{
  // Need this to for SOAP serialization of floats to work
  setlocale(LC_NUMERIC, "C");

  //start a session
  qi::Application app(argc, argv);

  ros::init(argc, argv, "naoqi_dcm_driver");

  ros::NodeHandle nh("~");
  if(!ros::master::check())
  {
    ROS_ERROR("Could not contact master!\nQuitting... ");
    return -1;
  }

  // Load Params from Parameter Server
  int pport = 9559;
  std::string pip = "127.0.0.1";
  std::string roscore_ip = "127.0.0.1";
  std::string network_interface = "eth0";
  nh.getParam("RobotIP", pip);
  nh.getParam("RobotPort", pport);
  nh.getParam("DriverBrokerIP", roscore_ip);
  nh.getParam("network_interface", network_interface);
  setMasterURINet( "http://"+roscore_ip+":11311", network_interface);

  //create a session
  qi::SessionPtr session = qi::makeSession();
  try
  {
    std::stringstream strstr;
    strstr << "tcp://" << pip << ":" << pport;
    ROS_INFO_STREAM("Connecting to " << pip << ":" << pport);
    session->connect(strstr.str()).wait();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("Cannot connect to session, %s", e.what());
    session->close();
    return -1;
  }

  // if (session->connected)
  // {
  //   ROS_ERROR("Cannot connect to session");
  //   session->close();
  //   return -1;
  // }

  // Deal with ALBrokerManager singleton (add your broker into NAOqi)
  boost::shared_ptr<Robot> robot = boost::make_shared<Robot>(session);

  // stop ALTouch service to prevent the robot shaking
  try
  {
    qi::AnyObject touch_proxy = session->service("ALTouch").value();
    touch_proxy.call<void>("exit");
    ROS_INFO_STREAM("Naoqi Touch service is shut down");
  }
  catch (const std::exception& e)
  {
    ROS_DEBUG("Did not stop ALTouch: %s", e.what());
  }

  // stop AutonomousLife service to prevent the robot shaking
  try
  {
    qi::AnyObject life_proxy = session->service("ALAutonomousLife").value();
    if (life_proxy.call<std::string>("getState") != "disabled")
    {
      life_proxy.call<void>("setState", "disabled");
      ROS_INFO_STREAM("Shutting down Naoqi AutonomousLife ...");
      ros::Duration(2.0).sleep();
    }
  }
  catch (const std::exception& e)
  {
    ROS_DEBUG("Did not stop AutonomousLife: %s", e.what());
  }

  session->registerService("naoqi_dcm_driver", robot);
  ros::Duration(0.1).sleep();

  if (!robot->connect())
  {
    session->close();
    return 0;
  }

  // Run the spinner in a separate thread to prevent lockups
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Run the main Loop
  robot->run();

  //release stiffness and stop correctly
  robot->stopService();

  //close the session
  session->close();
  spinner.stop();

  return 0;
}
