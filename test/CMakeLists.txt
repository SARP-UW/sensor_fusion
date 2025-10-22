###############################
# Test Executable (run_tests) #
###############################

# Define the executable that will run all unit tests.
# It includes the test runner's entry point (main_test.cpp) and all
# individual test suite files (*_test.cpp).
add_executable(run_tests main_test.cpp test_DataIngestion.cpp)

# Link the test runner against:
# 1. GTest::gtest: The GoogleTest framework.
# 2. fusion_lib: Our core application library (the code we want to test).
target_link_libraries(run_tests PRIVATE GTest::gtest fusion_lib)