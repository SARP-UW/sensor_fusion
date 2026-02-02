###############################
# Test Executable (run_tests) #
###############################

add_executable(run_tests main_test.cpp test_DataIngestion.cpp test_Eigen.cpp test_Ekf.cpp)
target_link_libraries(run_tests PRIVATE GTest::gtest fusion_lib)