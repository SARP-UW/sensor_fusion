####################################################
# Test Executable (run_tests)
####################################################

add_executable(run_tests main_test.cpp test_SensorData.cpp test_DataGenerator.cpp test_DataIngestion.cpp test_EkfInit.cpp test_EkfPredict.cpp test_EkfUpdate.cpp test_EkfNumerical.cpp test_EkfMekf.cpp test_FlightPhase.cpp test_EkfBias.cpp test_Eigen.cpp ../src/DataGenerator.cpp)
target_link_libraries(run_tests PRIVATE GTest::gtest fusion_lib)
