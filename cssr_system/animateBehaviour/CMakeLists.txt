# cmake_minimum_required(VERSION 3.0.2)
# project(cssr_system)

# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   roslib
#   std_msgs
#   std_srvs
#   trajectory_msgs
#   geometry_msgs
#   message_generation
#   actionlib
#   rostest
# )

# ## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   SetActivation.srv
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   std_srvs
# )

# ## Declare a catkin package
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cssr_system
#  CATKIN_DEPENDS roscpp std_msgs std_srvs trajectory_msgs geometry_msgs message_runtime
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

# ## Declare a C++ executable
# add_executable(${PROJECT_NAME}_animateBehaviour src/animateBehaviour.cpp src/animateBehaviourConfigAndService.cpp)
# set_target_properties(${PROJECT_NAME}_animateBehaviour PROPERTIES OUTPUT_NAME animateBehaviour PREFIX "")
# target_link_libraries(${PROJECT_NAME}_animateBehaviour ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(animateBehaviour src/animateBehaviourApplication.cpp src/animateBehaviourImplementation.cpp)
set_target_properties(animateBehaviour PROPERTIES OUTPUT_NAME animateBehaviour PREFIX "")
target_link_libraries(animateBehaviour ${catkin_LIBRARIES})
target_include_directories(animateBehaviour PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(animateBehaviour ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ test executable for physical robot
# add_executable(${PROJECT_NAME}_physicalRobotUnitTest src/physicalRobotUnitTest.cpp src/animateBehaviourConfigAndService.cpp)
# set_target_properties(${PROJECT_NAME}_physicalRobotUnitTest PROPERTIES OUTPUT_NAME physicalRobotUnitTest PREFIX "")
# target_link_libraries(${PROJECT_NAME}_physicalRobotUnitTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

# ## Declare a C++ test executable for simulator
# add_executable(${PROJECT_NAME}_simulatorUnitTest src/simulatorUnitTest.cpp src/animateBehaviourConfigAndService.cpp)
# set_target_properties(${PROJECT_NAME}_simulatorUnitTest PROPERTIES OUTPUT_NAME simulatorUnitTest PREFIX "")
# target_link_libraries(${PROJECT_NAME}_simulatorUnitTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
