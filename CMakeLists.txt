cmake_minimum_required(VERSION 3.14)

#########################
# Project Configuration #
#########################

project(SensorFusion VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

######################################
# Dependency Management (GoogleTest) #
######################################

include(FetchContent)
FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG release-1.12.1)
FetchContent_MakeAvailable(googletest)

########################
# Math Library (Eigen) #
########################

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen3)

##############################
# Build & Test Configuration #
##############################

enable_testing()
add_subdirectory(src)
add_subdirectory(test)
add_test(NAME run_all_tests COMMAND run_tests)
set_tests_properties(run_all_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
