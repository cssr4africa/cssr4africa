# Add executables with the separated source files
add_executable(robotLocalization src/robotLocalizationApplication.cpp src/robotLocalizationImplementation.cpp)
# Set properties for the executable
set_target_properties(robotLocalization PROPERTIES OUTPUT_NAME robotLocalization PREFIX "")
# Link the required libraries
target_link_libraries(robotLocalization ${catkin_LIBRARIES} ${aruco_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${JSONCPP_LIBRARIES} yaml-cpp)
# Include the necessary directories
target_include_directories(robotLocalization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# Add dependencies
add_dependencies(robotLocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})