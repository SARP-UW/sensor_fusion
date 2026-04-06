#include "gtest/gtest.h"
#include "DataIngestion.hpp"
#include "DataGenerator.hpp"
#include <cstdio>

class DataIngestionTest : public ::testing::Test
{
protected:
    static const std::string kTestFile;

    static void SetUpTestSuite()
    {
        DataGenerator::Config cfg;
        cfg.duration_sec = 15.0;
        cfg.output_filename = kTestFile;
        DataGenerator gen(cfg);
        gen.generate();
    }

    static void TearDownTestSuite()
    {
        std::remove(kTestFile.c_str());
    }
};

const std::string DataIngestionTest::kTestFile = "test_flight_data.bin";

TEST_F(DataIngestionTest, ShouldParseSyntheticFile)
{
    DataIngestion parser;
    ParsedData data = parser.loadFromFile(kTestFile);

    EXPECT_GT(data.imu_data.size(), 1000u);

    EXPECT_GT(data.baro_data.size(), 100u);

    EXPECT_GT(data.gps_data.size(), 50u);

    EXPECT_GT(data.mag_data.size(), 200u);

    if (!data.imu_data.empty())
    {
        EXPECT_NEAR(data.imu_data[0].timestamp_sec, 0.0, 0.001);
        EXPECT_EQ(data.imu_data[0].sensor_id, 0);
    }
}