add_executable(robotNavigationTest src/robotNavigationTestApplication.cpp src/robotNavigationTestImplementation.cpp )

set_target_properties(robotNavigationTest PROPERTIES OUTPUT_NAME robotNavigationTest PREFIX "" CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON )

target_link_libraries(robotNavigationTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${OpenCV_LIBRARIES} gtest gtest_main pthread )

target_include_directories(robotNavigationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} )

add_dependencies(robotNavigationTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cssr_system_generate_messages_cpp )

# Add compile definitions
target_compile_definitions(robotNavigationTest PRIVATE ROS=1 )

# Add the robotNavigationDriver executable
add_executable(robotNavigationDriver src/robotNavigationDriver.cpp )

set_target_properties(robotNavigationDriver PROPERTIES OUTPUT_NAME robotNavigationDriver PREFIX "" CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON )

target_link_libraries(robotNavigationDriver ${catkin_LIBRARIES} )

target_include_directories(robotNavigationDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} )

add_dependencies(robotNavigationDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )