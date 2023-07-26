cmake_minimum_required(VERSION 3.0.2)
project(pepper_diagnostic_routines)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

##Needed for ros packages
catkin_package(CATKIN_DEPENDS rospy naoqi_dcm_driver)
catkin_package()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    actionlib
    control_msgs
)

find_package(OpenCV REQUIRED)
    catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

# # action lib 
# find_package(catkin REQUIRED genmsg actionlib_msgs)
# generate_messages(DEPENDENCIES actionlib_msgs)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   ${catkin_INCLUDE_DIRS}
   include
)

#Install data files
install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

add_executable       (${PROJECT_NAME}_depth_camera src/depth_camera.cpp)
add_executable       (${PROJECT_NAME}_bottom_camera src/bottom_camera.cpp)
add_executable       (${PROJECT_NAME}_front_camera src/front_camera.cpp)
add_executable       (${PROJECT_NAME}_goToPosition src/goToPosition.cpp)
add_executable       (${PROJECT_NAME}_handTouch src/handTouch.cpp)
add_executable       (${PROJECT_NAME}_headTouch src/headTouch.cpp)
add_executable       (${PROJECT_NAME}_goToPositionMimo src/goToPositionMimo.cpp)
add_executable       (${PROJECT_NAME}_front_sonar src/front_sonar.cpp)
add_executable       (${PROJECT_NAME}_back_sonar src/back_sonar.cpp)
add_executable       (${PROJECT_NAME}_laser src/laser.cpp)

############################ Actuators control ################################### 
add_executable       (${PROJECT_NAME}_right_arm_control src/right_arm_control.cpp)
add_executable       (${PROJECT_NAME}_left_arm_control src/left_arm_control.cpp)
add_executable       (${PROJECT_NAME}_head_control src/head_control.cpp)
add_executable       (${PROJECT_NAME}_pelvis_control src/pelvis_control.cpp)
add_executable       (${PROJECT_NAME}_right_hand_control src/right_hand_control.cpp)
add_executable       (${PROJECT_NAME}_left_hand_control src/left_hand_control.cpp)




set_target_properties(${PROJECT_NAME}_depth_camera PROPERTIES OUTPUT_NAME depth_camera PREFIX "")
set_target_properties(${PROJECT_NAME}_bottom_camera PROPERTIES OUTPUT_NAME bottom_camera PREFIX "")
set_target_properties(${PROJECT_NAME}_front_camera PROPERTIES OUTPUT_NAME front_camera PREFIX "")
set_target_properties(${PROJECT_NAME}_goToPosition PROPERTIES OUTPUT_NAME goToPosition PREFIX "")
set_target_properties(${PROJECT_NAME}_handTouch PROPERTIES OUTPUT_NAME handTouch PREFIX "")
set_target_properties(${PROJECT_NAME}_headTouch PROPERTIES OUTPUT_NAME headTouch PREFIX "")
set_target_properties(${PROJECT_NAME}_goToPositionMimo PROPERTIES OUTPUT_NAME goToPositionMimo PREFIX "")
set_target_properties(${PROJECT_NAME}_front_sonar PROPERTIES OUTPUT_NAME front_sonar PREFIX "")
set_target_properties(${PROJECT_NAME}_back_sonar PROPERTIES OUTPUT_NAME back_sonar PREFIX "")
set_target_properties(${PROJECT_NAME}_laser PROPERTIES OUTPUT_NAME laser PREFIX "")

############################ Actuators control ################################### 
set_target_properties(${PROJECT_NAME}_right_arm_control PROPERTIES OUTPUT_NAME right_arm_control PREFIX "")
set_target_properties(${PROJECT_NAME}_left_arm_control PROPERTIES OUTPUT_NAME left_arm_control PREFIX "")
set_target_properties(${PROJECT_NAME}_head_control PROPERTIES OUTPUT_NAME head_control PREFIX "")
set_target_properties(${PROJECT_NAME}_pelvis_control PROPERTIES OUTPUT_NAME pelvis_control PREFIX "")
set_target_properties(${PROJECT_NAME}_right_hand_control PROPERTIES OUTPUT_NAME right_hand_control PREFIX "")
set_target_properties(${PROJECT_NAME}_left_hand_control PROPERTIES OUTPUT_NAME left_hand_control PREFIX "")



target_link_libraries(${PROJECT_NAME}_depth_camera ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_bottom_camera ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_front_camera ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_goToPosition ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_handTouch ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_headTouch ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_goToPositionMimo ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_front_sonar ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_back_sonar ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_laser ${catkin_LIBRARIES})

############################ Actuators control ################################### 
target_link_libraries(${PROJECT_NAME}_right_arm_control ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_left_arm_control ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_head_control ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_pelvis_control ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_right_hand_control ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_left_hand_control ${catkin_LIBRARIES})



