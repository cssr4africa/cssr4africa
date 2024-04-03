cmake_minimum_required(VERSION 3.0.2)
project(pepper_interface_tests)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Option to build for the simulator, which would not require naoqi_driver and specific definitions
option(SIMULATOR "Build for simulator without naoqi_driver and PEPPER_ROBOT definitions" OFF)

# Find catkin and system dependencies

# Conditionally find naoqi_driver unless building for the simulator
if(NOT SIMULATOR)
  message(STATUS "Building for Pepper robot")
  find_package(catkin REQUIRED COMPONENTS roscpp
  roslib
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  actionlib
  control_msgs 
  naoqi_driver) 
  add_definitions(-DPEPPER_ROBOT)
else()
  message(STATUS "Building for simulator")
  find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  actionlib
  control_msgs)
endif()

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

# Define catkin package dependencies conditionally based on whether building for the simulator or not
if(NOT SIMULATOR)
  catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs naoqi_driver
  )
else()
  catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
  )
endif()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable for actuator test
add_executable(${PROJECT_NAME}_actuatorTest src/actuatorTestImplementation.cpp src/actuatorTestApplication.cpp)
set_target_properties(${PROJECT_NAME}_actuatorTest PROPERTIES OUTPUT_NAME actuatorTest PREFIX "")
target_link_libraries(${PROJECT_NAME}_actuatorTest ${catkin_LIBRARIES})

# Declare a C++ executable for sensor test
add_executable(${PROJECT_NAME}_sensorTest src/sensorTestImplementation.cpp src/sensorTestApplication.cpp)
set_target_properties(${PROJECT_NAME}_sensorTest PROPERTIES OUTPUT_NAME sensorTest PREFIX "")
target_link_libraries(${PROJECT_NAME}_sensorTest ${OpenCV_LIBS} ${catkin_LIBRARIES})
