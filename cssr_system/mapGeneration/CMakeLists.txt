
# ## Declare a C++ executable
add_executable(mapGeneration src/mapGenerationApplication.cpp src/mapGenerationImplementation.cpp)
set_target_properties(mapGeneration PROPERTIES OUTPUT_NAME mapGeneration PREFIX "")
target_link_libraries(mapGeneration ${catkin_LIBRARIES})
target_include_directories(mapGeneration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(mapGeneration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


