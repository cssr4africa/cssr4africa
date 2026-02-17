cmake_minimum_required(VERSION 2.8.3)
project(utilities)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_package(
  INCLUDE_DIRS include
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# add_executable(${PROJECT_NAME}_cultureKnowledgeBaseExample src/cultureKnowledgeBaseApplication.cpp src/cultureKnowledgeBaseImplementation.cpp)
# set_target_properties(${PROJECT_NAME}_cultureKnowledgeBaseExample PROPERTIES OUTPUT_NAME cultureKnowledgeBaseExample  PREFIX "")
# target_link_libraries(${PROJECT_NAME}_cultureKnowledgeBaseExample ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}_environmentKnowledgeBaseExample src/environmentKnowledgeBaseApplication.cpp src/environmentKnowledgeBaseImplementation.cpp)
# set_target_properties(${PROJECT_NAME}_environmentKnowledgeBaseExample PROPERTIES OUTPUT_NAME environmentKnowledgeBaseExample  PREFIX "")
# target_link_libraries(${PROJECT_NAME}_environmentKnowledgeBaseExample ${catkin_LIBRARIES})
