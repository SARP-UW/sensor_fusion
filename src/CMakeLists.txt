####################################
# Application Library (fusion_lib) #
####################################

add_library(fusion_lib DataIngestion.cpp Ekf.cpp)
target_link_libraries(fusion_lib PUBLIC Eigen3::Eigen)
target_include_directories(fusion_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

###################################
# Main Executable (sensor_fusion) #
###################################

add_executable(sensor_fusion main.cpp)
target_link_libraries(sensor_fusion PRIVATE fusion_lib)

###################################
# Synthetic Data Generator (Tool) #
###################################

add_executable(generate_data mainGenerator.cpp DataGenerator.cpp)