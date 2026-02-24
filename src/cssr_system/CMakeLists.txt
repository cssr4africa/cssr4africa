cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
)

find_package(PkgConfig REQUIRED)

add_message_files(
  DIRECTORY
  sound_detection/msg
  FILES
  sound_detection_microphone_msg_file.msg
  
)

add_service_files(
  DIRECTORY
  conversation_management/srv
  FILES
  CreateCollection.srv
  Prompt.srv
)

add_service_files(
  DIRECTORY
  speech_event/srv
  FILES
  set_language.srv
  # set_enabled.srv
)

add_service_files(
  DIRECTORY
  xtts2/srv
  FILES
  RealtimeTTS.srv
)

add_action_files(
  DIRECTORY
  speech_event/action
  FILES
  recognise_speech.action
)

add_action_files(
  DIRECTORY
  xtts2/action
  FILES
  pizza_order.action
  english_stream_tts.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  actionlib_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp roslib rospy std_msgs std_srvs actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
