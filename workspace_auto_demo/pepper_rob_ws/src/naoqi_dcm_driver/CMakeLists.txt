cmake_minimum_required(VERSION 2.8.3)
project(naoqi_dcm_driver)
set(projectName
  naoqi_dcm_driver
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  hardware_interface
  controller_manager
  sensor_msgs
  naoqi_libqi
  naoqi_libqicore
  diagnostic_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

#Needed for ros packages
catkin_package()
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs tf std_msgs sensor_msgs hardware_interface controller_manager)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${NAOqi_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${projectName} 
  src/robot_driver.cpp 
  src/robot.cpp
  src/tools.cpp
  src/diagnostics.cpp
  src/memory.cpp
  src/dcm.cpp
  src/motion.cpp
  include/naoqi_dcm_driver/robot.hpp
  include/naoqi_dcm_driver/tools.hpp
  include/naoqi_dcm_driver/diagnostics.hpp
  include/naoqi_dcm_driver/memory.hpp
  include/naoqi_dcm_driver/dcm.hpp
  include/naoqi_dcm_driver/motion.hpp
)

target_link_libraries(${projectName}
  ${catkin_LIBRARIES}
  ${naoqi_libqi_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${projectName} 
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${projectName}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
