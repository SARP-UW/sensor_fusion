####################################
# Application Library (fusion_lib) #
####################################

# Define a static library "fusion_lib" containing the core application logic.
# All .cpp files that define the library's functionality should be added here.
add_library(fusion_lib DataIngestion.cpp)

# Expose the 'src' directory as a public include path for 'fusion_lib'.
# This allows any target that links against 'fusion_lib' to find its
# public headers (e.g., #include "DataIngestion.hpp").
target_include_directories(fusion_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

###################################
# Main Executable (sensor_fusion) #
###################################

# Define the main application executable
add_executable(sensor_fusion main.cpp)

# Link the main executable against our core logic library.
# This makes all functions from 'fusion_lib' available to 'sensor_fusion'.
target_link_libraries(sensor_fusion PRIVATE fusion_lib)