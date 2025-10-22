cmake_minimum_required(VERSION 3.14)

#########################
# Project Configuration #
#########################

# Define the project name and version
project(SensorFusion VERSION 1.0)

# Enforce C++17 standard across the project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

######################################
# Dependency Management (GoogleTest) #
######################################

# Enable FetchContent for in-build dependency management
include(FetchContent)

# Declare the GoogleTest dependency from its Git repository
FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG release-1.12.1)

# Download and configure GoogleTest (if not already present)
FetchContent_MakeAvailable(googletest)

##############################
# Build & Test Configuration #
##############################

# Enable CTest support for running tests
enable_testing()

# Add build targets from our source and test directories
add_subdirectory(src)
add_subdirectory(test)

# Define a CTest test case that executes our 'run_tests' binary
add_test(NAME run_all_tests COMMAND run_tests)

# Set the working directory for the test runner executable, as it's
# built inside the 'test/' subdirectory of the build tree.
set_tests_properties(run_all_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
