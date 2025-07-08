cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  angles
  aruco
  control_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(jsoncpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  behaviorController/msg
  FILES
  overtAttentionMode.msg
)

add_message_files(
  DIRECTORY
  face_detection/msg
  FILES
  face_detection_msg_file.msg
)

add_message_files(
  DIRECTORY
  gestureExecution/msg
  FILES
  Gesture.msg
)

add_message_files(
  DIRECTORY
  overtAttention/msg
  FILES
  Mode.msg
  Status.msg
)

add_message_files(
  DIRECTORY
  person_detection/msg
  FILES
  person_detection_msg_file.msg
)

add_message_files(
  DIRECTORY
  robotNavigation/msg
  FILES
  Goal.msg
)

add_message_files(
  DIRECTORY
  sound_detection/msg
  FILES
  sound_detection_microphone_msg_file.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  animateBehaviour/srv
  FILES
  setActivation.srv
)

add_service_files(
  DIRECTORY
  behaviorController/srv
  FILES
  animateBehaviorSetActivation.srv
  gestureExecutionPerformGesture.srv
  overtAttentionSetMode.srv
  robotLocalizationSetPose.srv
  robotNavigationSetGoal.srv
  speechEventSetStatus.srv
  speechEventSetLanguage.srv
  tabletEventPromptAndGetResponse.srv
  textToSpeechSayText.srv
)

add_service_files(
  DIRECTORY
  gestureExecution/srv
  FILES
  performGesture.srv
)

add_service_files(
  DIRECTORY
  overtAttention/srv
  FILES
  setMode.srv
)

add_service_files(
  DIRECTORY 
  robotLocalization/srv
  FILES
  resetPose.srv
  setPose.srv
)

add_service_files(
  DIRECTORY
  robotNavigation/srv
  FILES
  setGoal.srv
)

add_service_files(
  DIRECTORY
  speech_event/srv
  FILES
  set_language.srv
  set_enabled.srv
)
  
add_service_files(
  DIRECTORY 
  text_to_speech/srv
  FILES
  TTS.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS animateBehaviour behaviorController gestureExecution mapGeneration overtAttention robotLocalization robotNavigation
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles cv_bridge geometry_msgs image_transport message_runtime nav_msgs roscpp roslib rospy sensor_msgs std_msgs std_srvs tf tf2 tf2_ros trajectory_msgs
  DEPENDS OpenCV yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  animateBehaviour/include
  behaviorController/include
  gestureExecution/include
  mapGeneration/include
  overAttention/include
  robotLocalization/include
  robotNavigation/include
  ${aruco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_subdirectory(animateBehaviour)
add_subdirectory(behaviorController)
add_subdirectory(gestureExecution)
add_subdirectory(mapGeneration)
add_subdirectory(overtAttention)
add_subdirectory(robotLocalization)
add_subdirectory(robotNavigation)
