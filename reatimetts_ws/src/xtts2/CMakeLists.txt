cmake_minimum_required(VERSION 3.0.2)
project(kinyarwanda_tts)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TTSRequest.srv
)

# Add action files
add_action_files(
  DIRECTORY action
  FILES TTS.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime actionlib actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

