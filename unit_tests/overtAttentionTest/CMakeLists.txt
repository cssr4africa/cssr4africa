# Add the executables
add_executable        (overtAttentionTest src/overtAttentionTestApplication.cpp src/overtAttentionTestImplementation.cpp)
set_target_properties (overtAttentionTest PROPERTIES OUTPUT_NAME overtAttentionTest PREFIX "")
target_link_libraries (overtAttentionTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
target_include_directories(overtAttentionTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(overtAttentionTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable        (overtAttentionTestDriver src/overtAttentionTestDriver.cpp)
set_target_properties (overtAttentionTestDriver PROPERTIES OUTPUT_NAME overtAttentionTestDriver PREFIX "")
target_link_libraries (overtAttentionTestDriver ${catkin_LIBRARIES})
target_include_directories(overtAttentionTestDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(overtAttentionTestDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})