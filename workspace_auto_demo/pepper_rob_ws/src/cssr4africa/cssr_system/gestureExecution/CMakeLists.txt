
# Add the executables
add_executable        (gestureExecution src/gestureExecutionImplementation.cpp src/gestureExecutionApplication.cpp src/pepperKinematicsUtilitiesImplementation.cpp)
set_target_properties (gestureExecution PROPERTIES OUTPUT_NAME gestureExecution PREFIX "")
target_link_libraries (gestureExecution ${catkin_LIBRARIES})
target_include_directories(gestureExecution PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(gestureExecution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})