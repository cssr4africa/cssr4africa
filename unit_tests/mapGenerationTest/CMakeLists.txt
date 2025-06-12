# Find GTest for this target
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Declare the main test executable
add_executable(mapGenerationTest 
    src/mapGenerationTestApplication.cpp 
    src/mapGenerationTestImplementation.cpp
)

# Set target properties
set_target_properties(mapGenerationTest PROPERTIES 
    OUTPUT_NAME mapGenerationTest 
    PREFIX ""
)

# Link libraries
target_link_libraries(mapGenerationTest 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${GTEST_LIBRARIES} 
    pthread
)

# Include directories
target_include_directories(mapGenerationTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${catkin_INCLUDE_DIRS}
)

# Dependencies
add_dependencies(mapGenerationTest 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

# Add the test
add_test(NAME MapGenerationTests COMMAND mapGenerationTest)