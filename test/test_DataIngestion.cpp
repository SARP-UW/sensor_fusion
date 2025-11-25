#include "gtest/gtest.h"
#include "DataIngestion.hpp"
#include <fstream>

// A helper to ensure the file exists before testing.
bool fileExists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

TEST(DataIngestionTest, ShouldParseSyntheticFile) {
    std::string filename = "flight_data.bin";
    
    // Sanity Check: Run the generator.
    ASSERT_TRUE(fileExists(filename)) << "Run ./src/generate_data first!";

    // Run the code under test.
    DataIngestion parser;
    ParsedData data = parser.loadFromFile(filename);
    
    // We generated 15 seconds of data at 100Hz -> ~1500 IMU packets.
    EXPECT_GT(data.imu_data.size(), 1000); 
    
    // We generated Baro data at 10Hz -> ~150 Baro packets.
    EXPECT_GT(data.baro_data.size(), 100);

    // Check the first packet.
    if (!data.imu_data.empty()) {
        EXPECT_NEAR(data.imu_data[0].timestamp_sec, 0.0, 0.001);
        EXPECT_EQ(data.imu_data[0].sensor_id, 0);
    }
}