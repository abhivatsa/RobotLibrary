# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(RoboticsLibrary VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Optionally, set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Include FetchContent for managing external dependencies
include(FetchContent)

# FetchContent declaration for nlohmann/json
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.2 # Specify the desired version
)

# Make the content available
FetchContent_MakeAvailable(json)

# Define the library as header-only
add_library(RoboticsLibrary INTERFACE)

# Specify include directories
target_include_directories(RoboticsLibrary INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link nlohmann/json to RoboticsLibrary
target_link_libraries(RoboticsLibrary INTERFACE nlohmann_json::nlohmann_json)

# Define compile features
target_compile_features(RoboticsLibrary INTERFACE cxx_std_17)

# Add install rules
include(GNUInstallDirs)

# Install headers
install(DIRECTORY include/robotLibrary/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotLibrary
)

# Install ConfigLoader.h (but not config.json)
install(FILES config/ConfigLoader.h
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/roboticslibrary/config
)

# Install the library target without exporting the config
install(TARGETS RoboticsLibrary
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Remove the export and installation of RoboticsLibraryConfig
# Commented out since we're not generating or installing these files
# install(EXPORT RoboticsLibraryConfig
#     FILE RoboticsLibraryConfig.cmake
#     NAMESPACE RoboticsLibrary::
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/RoboticsLibrary
# )

# Remove package configuration since we're not generating the config files
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/RoboticsLibraryConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
# )

# Remove installation of the config files
# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/RoboticsLibraryConfig.cmake"
#     "${CMAKE_CURRENT_BINARY_DIR}/RoboticsLibraryConfigVersion.cmake"
#     DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/RoboticsLibrary
# )

# Optionally, set up testing (commented out since tests are not included)
# enable_testing()
# add_subdirectory(tests)

# Optionally, set up examples (commented out since examples are not included)
# add_subdirectory(examples)
