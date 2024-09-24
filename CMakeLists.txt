cmake_minimum_required(VERSION 3.10)
project(RobotLibrary)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create library
add_library(RobotLibrary STATIC ${SOURCES})

# Google Test
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add executable for tests
add_executable(RobotLibraryTests tests/KinematicsTests.cpp)
target_link_libraries(RobotLibraryTests RobotLibrary GTest::GTest GTest::Main)

# Link math library if it's a separate library
# target_link_libraries(RobotLibraryTests RobotLibrary MathLibrary)

# Add tests
add_test(NAME RobotLibraryTests COMMAND RobotLibraryTests)

