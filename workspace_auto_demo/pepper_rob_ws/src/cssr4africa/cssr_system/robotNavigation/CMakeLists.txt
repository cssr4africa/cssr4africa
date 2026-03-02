
# ## Declare a C++ executable
add_executable(robotNavigation src/robotNavigationApplication.cpp src/robotNavigationImplementation.cpp)
set_target_properties(robotNavigation PROPERTIES OUTPUT_NAME robotNavigation PREFIX "")
target_link_libraries(robotNavigation ${catkin_LIBRARIES})
target_include_directories(robotNavigation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(robotNavigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


