# CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(RoboticsLibrary VERSION 1.0.0 LANGUAGES CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directories
include_directories(include)

# Add nlohmann/json
# If using package manager
find_package(nlohmann_json 3.2.0 REQUIRED)

# Alternatively, if added manually
# include_directories(include/RoboticsLibrary/external)

# Gather all headers
file(GLOB_RECURSE HEADERS "include/RoboticsLibrary/*.h")

# Gather all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the library
add_library(RoboticsLibrary STATIC
    ${HEADERS}
    ${SOURCES}
)

# Link libraries
target_link_libraries(RoboticsLibrary PUBLIC nlohmann_json::nlohmann_json)

# Specify include directories for the library target
target_include_directories(RoboticsLibrary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enable testing and add subdirectories as needed
enable_testing()
add_subdirectory(tests)

# Add examples if any
add_subdirectory(examples)
