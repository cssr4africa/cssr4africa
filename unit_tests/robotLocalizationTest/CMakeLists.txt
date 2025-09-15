
add_executable(robotLocalizationTest src/robotLocalizationTestApplication.cpp src/robotLocalizationTestImplementation.cpp)

set_target_properties(robotLocalizationTest PROPERTIES OUTPUT_NAME robotLocalizationTest PREFIX "" CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)

target_link_libraries(robotLocalizationTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${OpenCV_LIBRARIES} gtest gtest_main pthread)

target_include_directories(robotLocalizationTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

add_dependencies(robotLocalizationTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp cssr_system_generate_messages_cpp)

# Add compile definitions
target_compile_definitions(robotLocalizationTest PRIVATE ROS=1)

# robotLocalizationTestDriver executable
add_executable(robotLocalizationTestDriver src/robotLocalizationTestDriver.cpp)

set_target_properties(robotLocalizationTestDriver PROPERTIES OUTPUT_NAME robotLocalizationTestDriver PREFIX "" CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)

target_link_libraries(robotLocalizationTestDriver ${catkin_LIBRARIES})

target_include_directories(robotLocalizationTestDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

add_dependencies(robotLocalizationTestDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)