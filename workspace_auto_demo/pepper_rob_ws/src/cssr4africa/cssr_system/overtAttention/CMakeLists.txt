# Add the executables
add_executable        (overtAttention src/overtAttentionImplementation.cpp src/overtAttentionApplication.cpp)
set_target_properties (overtAttention PROPERTIES OUTPUT_NAME overtAttention PREFIX "")
target_link_libraries (overtAttention ${catkin_LIBRARIES})
target_include_directories(overtAttention PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(overtAttention ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})