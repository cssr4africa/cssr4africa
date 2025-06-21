cmake_minimum_required(VERSION 3.0.2)
project(cssr_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  actionlib
  control_msgs
  geometry_msgs
  nav_msgs
  tf
  angles
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV REQUIRED)
#     catkin_package(
#     CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
# )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

# file(GLOB MSG_FILES ${PROJECT_NAME}/gestureExecution/msg/*.msg)
# file(GLOB SRV_FILES ${PROJECT_NAME}/gestureExecution/srv/*.srv)


# catkin_python_setup()  # Needs to be called before generate_messages()

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  gestureExecution/msg
  FILES
  Gesture.msg
  # Message2.msg
)

add_message_files(
  DIRECTORY
  overtAttention/msg
  FILES
  Mode.msg
  Status.msg
  # Message2.msg
)

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
  # Message2.msg
)

## Add service and message files
add_message_files(
  DIRECTORY
  person_detection/msg
  FILES
  person_detection_msg_file.msg
)



## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  gestureExecution/srv
  FILES
  performGesture.srv
  # Service2.srv
)

add_service_files(
  DIRECTORY
  overtAttention/srv
  FILES
  setMode.srv
  # Service2.srv
)

add_service_files(
  DIRECTORY
  animateBehaviour/srv
  FILES
  setActivation.srv
  # Service2.srv
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
  speech_event/srv
  FILES
  set_language.srv
  set_enabled.srv
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
  std_msgs  # Or other packages containing msgs
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 INCLUDE_DIRS gestureExecution overtAttention animateBehaviour mapGeneration behaviorController
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
    catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs rospy sensor_msgs roslib nav_msgs tf angles message_runtime
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
gestureExecution/include
overAttention/include
animateBehaviour/include
mapGeneration/include
behaviorController/include
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(gestureExecution)
add_subdirectory(overtAttention)
add_subdirectory(animateBehaviour)
add_subdirectory(mapGeneration)
add_subdirectory(behaviorController)