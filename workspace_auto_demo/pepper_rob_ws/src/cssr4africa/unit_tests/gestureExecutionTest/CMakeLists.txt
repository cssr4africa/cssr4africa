
# Add the executables
add_executable        (gestureExecutionTest src/gestureExecutionTestApplication.cpp src/gestureExecutionTestImplementation.cpp)
set_target_properties (gestureExecutionTest PROPERTIES OUTPUT_NAME gestureExecutionTest PREFIX "")
target_link_libraries (gestureExecutionTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
target_include_directories(gestureExecutionTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(gestureExecutionTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add the executables for the stub and driver
add_executable        (gestureExecutionTestStub src/gestureExecutionTestStub.cpp)
set_target_properties (gestureExecutionTestStub PROPERTIES OUTPUT_NAME gestureExecutionTestStub PREFIX "")
target_link_libraries (gestureExecutionTestStub ${catkin_LIBRARIES})
target_include_directories(gestureExecutionTestStub PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(gestureExecutionTestStub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add the executables for the stub and driver
add_executable        (gestureExecutionTestDriver src/gestureExecutionTestDriver.cpp)
set_target_properties (gestureExecutionTestDriver PROPERTIES OUTPUT_NAME gestureExecutionTestDriver PREFIX "")
target_link_libraries (gestureExecutionTestDriver ${catkin_LIBRARIES})
target_include_directories(gestureExecutionTestDriver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
add_dependencies(gestureExecutionTestDriver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})